#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import cv2
import numpy as np
import time

from apple_interfaces.msg import CameraConsensus
from apple_interfaces.srv import CloudScan
from apple_interfaces.srv import UpdateBin

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('bin_scanner')

        camera_sn = "GDS871PBAA7110621" # Will come from hardware manager
        self.camera_info = None # Visual camera info
        self.window_name = "Camera"
        self.confidence_threshold = 0.7
        self.qr_frame_id = "QR_Code_1"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_ready = False
        self.tf_timer = self.create_timer(0.2, self.check_tf_ready)

        self.group_1 = MutuallyExclusiveCallbackGroup()

        # Create a subscriber to the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            Image,
            f'/arm1_cam/{camera_sn}/color/image_raw', 
            self.image_callback,
            10,
            callback_group=self.group_1
        )

        # Point Cloud processor clinet
        self.cloud_client = self.create_client(
            CloudScan, "/arm1/cloud_scan"
        )

        self.bin_client = self.create_client(
            UpdateBin, "/update_bin"
        )

        self.trigger_service = self.create_service(
            Trigger,
            "/trigger_qr_scan",
            self.trigger_callback,
            callback_group=self.group_1
        )

        # self.bin_pose_pub = self.create_publisher(
        #     PoseStamped, "/temp_visual", 10
        # )

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.trigger_detected = False
        self.bin_updated = False

        self.get_logger().info("QR Detector Node Started")

        cv2.namedWindow("QR Detector", cv2.WINDOW_NORMAL)
        self.latest_frame = None
        # self.cv_timer = self.create_timer(0.1, self.showImage)
    def check_tf_ready(self):
        if self.tf_ready:
            return

        try:
            T_dummy_qr = self.tf_buffer.lookup_transform(
                "dummy_link",
                self.qr_frame_id,
                rclpy.time.Time()
            )

            self.L_dummy_qr = np.array([
                T_dummy_qr.transform.translation.x,
                T_dummy_qr.transform.translation.y,
                T_dummy_qr.transform.translation.z
            ])

            self.tf_ready = True
            self.get_logger().info("TF ready: dummy_link -> qr_frame")

            self.tf_timer.cancel()

        except TransformException:
            self.get_logger().debug("Waiting for TF tree...")
    def trigger_callback(self, request, response):
        self.trigger_detected = True
        start_time = self.get_clock().now()
        while not self.bin_updated:
            current_time = self.get_clock().now()
            if (current_time - start_time).nanoseconds / 1e9 > 20.0:
                response.success = False
                self.trigger_detected = False
                self.bin_updated = False
                return response
            time.sleep(0.1)
        response.success = True
        self.trigger_detected = False
        self.bin_updated = False
        return response

    def image_callback(self, msg):
        if not self.tf_ready:
            return
        # Convert ROS image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.trigger_detected:
            # Detect and decode the QR code
            value, points, _ = self.detector.detectAndDecode(frame)

            # Check if QR code is detected
            if value:
                # print(f'Detected QR Code: {value}')
                # Draw the bounding box around the QR code
                if points is not None:
                    points = points[0].astype(int)
                    for i in range(4):
                        cv2.line(frame, tuple(points[i]), tuple(points[(i+1)%4]), (0, 255, 0), 5)

                    qr_message = CameraConsensus()
                    qr_message.header.stamp = self.get_clock().now().to_msg()
                    qr_message.header.frame_id = "arm1_cam_color_frame"  # or camera frame

                    # Top-left and bottom-right corners
                    x1 = min(points[:, 0])
                    y1 = min(points[:, 1])
                    x2 = max(points[:, 0])
                    y2 = max(points[:, 1])

                    padding = 40  # pixels

                    height, width = frame.shape[:2]

                    x1 = max(0, x1 - padding)
                    y1 = max(0, y1 - padding)
                    x2 = min(width - 1, x2 + padding)
                    y2 = min(height - 1, y2 + padding)

                    qr_message.u1 = int(x1)
                    qr_message.v1 = int(y1)
                    qr_message.u2 = int(x2)
                    qr_message.v2 = int(y2)

                    scan_request = CloudScan.Request()
                    scan_request.pixel_locations = [qr_message]
                    scan_request.size = 1

                    future = self.cloud_client.call_async(scan_request)
                    future.add_done_callback(self.consensus_done)
        self.latest_frame = frame

    def consensus_done(self, future):
        try:
            response = future.result()
            if not response.success:
                raise Exception("Cloud Scan Fail")
            for pose in response.world_locations:
                L_base_qr = np.array([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                ])
                L_base_dummy = L_base_qr - self.L_dummy_qr

                bin_request = UpdateBin.Request()
                bin_request.new_pose = L_base_dummy

                future = self.bin_client.call_async(bin_request)
                future.add_done_callback(self.bin_scan_done)

        except Exception as e:
            self.get_logger().error(str(e))

    def bin_scan_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.bin_updated = True
                self.get_logger().info("Updated bin pose")
            else:
                self.get_logger().info("Unable to update bin pose")
        except Exception as e:
            self.get_logger().error(str(e))
    
    def showImage(self):
        # Show image
        if self.latest_frame is not None:
            cv2.imshow("QR Detector", self.latest_frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()

    # MultiThreadedExecutor with 8 worker threads
    executor = MultiThreadedExecutor(num_threads=4)

    # Add node to executor
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once()
            node.showImage()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()