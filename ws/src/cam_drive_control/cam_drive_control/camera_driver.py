import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image

import torch
import os
import copy
import numpy as np
from dataclasses import dataclass
import threading
import time

from ultralytics import YOLO

from apple_interfaces.msg import CameraConsensus
from apple_interfaces.srv import CloudScan
from apple_interfaces.action import VisionScan
from enum import Enum, auto

class PerceptionMode(Enum):
    IDLE = auto()
    APPLE = auto()
    QR = auto()

@dataclass
class ConsensusStruct:
    u1: int
    v1: int
    u2: int
    v2: int
    confidence: float

def consensusEqual(C1: ConsensusStruct, C2: ConsensusStruct):
    corner1 = np.abs(C1.u1 - C2.u1) < 1.0 and np.abs(C1.v1 - C2.v1) < 1.0
    corner2 = np.abs(C1.u2 - C2.u2) < 1.0 and np.abs(C1.v2 - C2.v2) < 1.0
    return (corner1 and corner2)

class CameraDriver(Node):
    """A ROS node that reads from the rgbd camera, runs a model, and fetches the points near the consensus with wasd pan control"""

    def __init__(self):
        super().__init__('camera_driver')

        self.declare_parameter('model_name', 'wsu-v9c.pt')

        self.apple_model_name = self.get_parameter('model_name').value

        if self.apple_model_name == "None":
            self.get_logger().warn("No model specified, returning raw image!")
            self.apple_model = None
        else:
            # Set to new model
            try:
                self.get_logger().info(f"Loading model: {self.apple_model_name}")

                model_path = os.path.join(
                    get_package_share_directory("cam_drive_control"),
                    'models',
                    self.apple_model_name
                )

                model = YOLO(model_path)

                self.apple_model = model

                self.get_logger().info("YOLO model loaded successfully.")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to load YOLO model: {e}"
                )

        self.qr_model = 1

        camera_sn = "GDS871PBAA7110621" # Will come from hardware manager
        self.camera_info = None # Visual camera info
        self.confidence_threshold = 0.7

        #==================VARIABLES===========================
        self.image_lock = threading.Lock()
        self.latest_image_raw = None
        self.results_lock = threading.Lock()
        self.results = None
        self.selected_lock = threading.Lock()
        self.selected_results = list()
        self.mode_lock = threading.Lock()
        self.mode = PerceptionMode.APPLE
        self.service_result = None
        self.service_lock = threading.Lock()
        #==================USER DEPENDENT VARIABLES===========
        self.cv_lock = threading.Lock()
        self.clicked_locations = list()
        #==================TOPICS==============================
        self.callback_group = ReentrantCallbackGroup()
        # Create a subscriber to the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            Image,
            f'/arm1_cam/{camera_sn}/color/image_raw', 
            self.image_callback,
            10
        )
        self.camera_consensus_client = self.create_client(
            CloudScan, "/arm1/cloud_scan",
            callback_group=self.callback_group
        )
        self.vision_action = ActionServer(
            self,
            VisionScan,
            "/cam_drive_control/vision_scan",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
    
    def image_callback(self, msg):
        with self.image_lock:
            self.latest_image_raw = self.convert_image(msg)

        if self.mode == PerceptionMode.APPLE:
            self.detect_apples()

        if self.mode == PerceptionMode.QR:
            self.detect_qr()
    
    def convert_image(self, msg):
        try:
            # Convert ROS2 Image message to NumPy array
            img_array = np.frombuffer(msg.data, dtype=np.uint8)
            img_bgr = None
            # Reshape based on encoding
            if msg.encoding == 'rgb8':
                img_array = img_array.reshape((msg.height, msg.width, 3))
                img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img_bgr = img_array.reshape((msg.height, msg.width, 3))
            elif msg.encoding == 'mono8':
                img_bgr = img_array.reshape((msg.height, msg.width))
            else:
                self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
                return None
            return img_bgr
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def detect_apples(self):
        with self.cv_lock:
            local_clicked = list(self.clicked_locations)
            self.clicked_locations.clear()

        iter_results = self.apple_model(self.latest_image_raw, verbose=False)

        with self.results_lock:
            self.results = iter_results[0]

        for u, v in local_clicked:

            for box in self.results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()


                if x1 <= u <= x2 and y1 <= v <= y2:

                    if float(box.conf[0]) < self.confidence_threshold:
                        return

                    self.get_logger().info(
                        f"Pixel Coordinates: u1:{x1}, v1{y1}, u1{x2}, v2{y2}"
                    )

                    newCons = ConsensusStruct(x1, y1, x2, y2, box.conf[0])
                    append = True
                    for cons in self.selected_results:
                        if consensusEqual(cons, newCons):
                            append = False
                    if append:
                        with self.selected_lock:
                            self.selected_results.append(newCons)

            return
        

    def detect_qr(self):
        return

    def handle_click(self, u, v):
        with self.cv_lock:
            self.clicked_locations.append((u,v))

    def get_image_raw(self):
        with self.image_lock:
            return self.latest_image_raw

    def get_image_results(self):
        with self.results_lock:
            return self.results

    def goal_callback(self, goal_request):
        if goal_request.order not in (
            VisionScan.Goal.APPLE_SCAN,
            VisionScan.Goal.QR_SCAN
        ):
            self.get_logger().warn("Invalid scan type requested.")
            return GoalResponse.REJECT

        if self.apple_model is None or self.qr_model is None:
            self.get_logger().warn("Scan requested on no inference.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        if goal.order == VisionScan.Goal.APPLE_SCAN:
            with self.selected_lock:
                local_results = list(self.selected_results)
                self.selected_results.clear()
        else:
            local_results = list()
            # Enable qr scanning and find consensus
        if len(local_results) == 0:
            goal_handle.abort()
            result = VisionScan.Result()
            result.status = "No selectable results"
            return result
        pixel_locations = list()
        for cons in local_results:
            cons_message = CameraConsensus()
            cons_message.header.stamp = self.get_clock().now().to_msg()
            cons_message.header.frame_id = "arm1_cam_color_frame"  # or camera frame

            # Pixel coordinates of pick
            cons_message.u1 = int(cons.u1)
            cons_message.v1 = int(cons.v1)
            cons_message.u2 = int(cons.u2)
            cons_message.v2 = int(cons.v2)

            pixel_locations.append(cons_message)
        
        scan_request = CloudScan.Request()
        scan_request.pixel_locations = pixel_locations
        scan_request.size = len(local_results)
        
        future = self.camera_consensus_client.call_async(scan_request)
        future.add_done_callback(self.service_callback)

        rate = self.create_rate(50)
        with self.service_lock:
            self.service_result = None
            while rclpy.ok():
                # cancellation support
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = VisionScan.Result()
                    result.status = "Canceled"
                    return result

                # check completion
                if self.service_result is not None:
                    break

                rate.sleep()

            scan_result = self.service_result
            self.service_result = None

        if not scan_result.success:
            goal_handle.abort()
            result = VisionScan.Result()
            result.status = "PointCloud scan failure"
            return result

        world_locations = scan_result.world_locations
        feedback_msg = VisionScan.Feedback()
        feedback_msg.success = True
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = VisionScan.Result()

        if goal.order == VisionScan.Goal.APPLE_SCAN:
            result.apples = world_locations
        else:
            result.qr_pose = world_locations[0]
        self.get_logger().info(f'Goal succeeded!')
        return result


    def service_callback(self, future):
        try:
            self.get_logger().info("Service complete")
            self.service_result = future.result()
        except Exception as e:
            self.get_logger().error(str(e))
            self.service_result = None