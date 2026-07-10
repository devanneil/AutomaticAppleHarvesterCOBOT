import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from ament_index_python.packages import get_package_share_directory
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
# import tf_transformations  # For quaternion to Euler conversion

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
import tf2_geometry_msgs

import torch
import os
import copy
import numpy as np
from dataclasses import dataclass
import threading
import time

from ultralytics import YOLO
from qreader import QReader


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
    depth_field: np.ndarray
    tf: TransformStamped

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

        self.qr_model = QReader()

        camera_sn = "GDS871PBAA7110621" # Will come from hardware manager
        self.camera_info = None # Visual camera info
        self.confidence_threshold = 0.7

        #==================VARIABLES===========================
        self.image_lock = threading.Lock()
        self.latest_image_raw = None
        self.image_count_since_last_state = 0
        self.results_lock = threading.Lock()
        self.results = None
        self.results_qr = None
        self.selected_lock = threading.Lock()
        self.selected_results = list()
        self.mode_lock = threading.Lock()
        self.mode = PerceptionMode.APPLE
        self.service_result = None
        self.service_lock = threading.Lock()
        self.cx = None
        self.cy = None
        self.fx = None
        self.fy = None
        #==================USER DEPENDENT VARIABLES===========
        self.cv_lock = threading.Lock()
        self.clicked_locations = list()
        #==================TOPICS==============================
        self.camera_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()
        # Create a subscriber to the /camera/image_raw topic
        self.vis_sub = Subscriber(self, Image, f'/arm1_cam/{camera_sn}/color/image_raw', callback_group=self.camera_group)
        self.depth_sub = Subscriber(self, Image, f'/arm1_cam/{camera_sn}/transformedDepth/image_raw', callback_group=self.camera_group)
        self.ts = ApproximateTimeSynchronizer([self.vis_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)
        self.cam_intrinsics_sub = self.create_subscription(
            CameraInfo,
            f'/arm1_cam/{camera_sn}/color/camera_info',
            self.cam_info_callback,
            10,
            callback_group=self.camera_group
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.vision_action = ActionServer(
            self,
            VisionScan,
            "/cam_drive_control/vision_scan",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_group
        )
    
    def image_callback(self, image_msg, depth_msg):
        with self.image_lock:
            self.latest_image_raw = self.convert_image(image_msg)
            self.image_count_since_last_state += 1

        if self.mode == PerceptionMode.APPLE:
            self.detect_apples(self.latest_image_raw, depth_msg)

        if self.mode == PerceptionMode.QR:
            self.detect_apples(self.latest_image_raw, depth_msg)

        # self.get_logger().info(
        #     f"mode={self.mode} images={self.image_count_since_last_state}"
        # )

    def cam_info_callback(self, msg):
        if self.cx is not None:
            return

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def convert_image(self, msg):
        try:
            if msg.encoding == "rgb8":
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img_array = img_array.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

            elif msg.encoding == "bgr8":
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                return img_array.reshape((msg.height, msg.width, 3))

            elif msg.encoding == "mono8":
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                return img_array.reshape((msg.height, msg.width))

            elif msg.encoding == "16UC1":
                img_array = np.frombuffer(msg.data, dtype=np.uint16)
                return img_array.reshape((msg.height, msg.width))

            else:
                self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
                return None

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return None

    def detect_apples(self, image, depth_ros_msg):
        with self.cv_lock:
            local_clicked = list(self.clicked_locations)
            self.clicked_locations.clear()

        iter_results = self.apple_model(image, verbose=False)
        if len(iter_results) == 0:
            return

        with self.results_lock:
            self.results = iter_results[0]

        for u, v in local_clicked:

            for box in self.results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()


                if x1 <= u <= x2 and y1 <= v <= y2:

                    if float(box.conf[0]) < self.confidence_threshold:
                        return

                    # self.get_logger().info(
                    #     f"Pixel Coordinates: u1:{x1}, v1{y1}, u1{x2}, v2{y2}"
                    # )
                    t_base_cam = self.tf_buffer.lookup_transform(
                        "base_link",
                        depth_ros_msg.header.frame_id,
                        depth_ros_msg.header.stamp
                    )
                    newCons = ConsensusStruct(x1, y1, x2, y2, box.conf[0],
                        self.convert_image(depth_ros_msg), t_base_cam)
                    append = True
                    for cons in self.selected_results:
                        if consensusEqual(cons, newCons):
                            append = False
                    if append:
                        with self.selected_lock:
                            self.selected_results.append(newCons)

            return
        

    def detect_qr(self, image, depth_ros_msg):
        detections = self.qr_model.detect(image, False)
        if len(detections) != 0:
            best = max(detections, key=lambda d: d["confidence"])
            x1, y1, x2, y2 = map(int, np.round(best["bbox_xyxy"]))

            padding = 40  # pixels

            height, width = image.shape[:2]

            x1 = max(0, x1 - padding)
            y1 = max(0, y1 - padding)
            x2 = min(width - 1, x2 + padding)
            y2 = min(height - 1, y2 + padding)
            t_base_cam = self.tf_buffer.lookup_transform(
                depth_ros_msg.header.frame_id,
                "base_link",
                depth_ros_msg.header.stamp
            )
            newCons = ConsensusStruct(x1, y1, x2, y2, box.conf[0], 
                self.convert_image(depth_ros_msg), t_base_cam)
            with self.results_lock:
                self.results_qr = newCons

    def handle_click(self, u, v):
        with self.cv_lock:
            self.clicked_locations.append((u,v))

    def get_image_raw(self):
        with self.image_lock:
            return self.latest_image_raw

    def get_image_results(self):
        with self.results_lock:
            return (
                copy.copy(self.results),
                copy.copy(self.results_qr)
            )

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
        if goal_request.order == VisionScan.Goal.APPLE_SCAN:
            if self.mode != PerceptionMode.APPLE:
                self.mode = PerceptionMode.APPLE

                #self.get_logger().info("Changing to apple mode")
        else:
            if self.mode != PerceptionMode.QR:
                self.mode = PerceptionMode.QR

                #self.get_logger().info("Changing to qr mode")

        with self.image_lock:
            self.image_count_since_last_state = 0
        with self.results_lock:
            self.results = None
            self.reults_qr = None    
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        rate = self.create_rate(50)
        while rclpy.ok():
            if self.image_count_since_last_state > 20:
                goal_handle.abort()
                result = VisionScan.Result()
                result.status = "No consensus found!"
                with self.results_lock:
                    self.results = None
                    self.reults_qr = None
                self.mode = PerceptionMode.APPLE
                return result
            with self.results_lock:
                if self.mode == PerceptionMode.APPLE and self.results is not None and self.cx is not None:
                    local_results = list(self.selected_results)
                    self.selected_results.clear()
                    break
                if self.mode == PerceptionMode.QR and self.results_qr is not None and self.cx is not None:
                    local_results = list(self.results_qr)
                    break

            rate.sleep()
        # Enable qr scanning and find consensus
        if len(local_results) == 0:
            goal_handle.abort()
            result = VisionScan.Result()
            result.status = "No selectable results"
            with self.results_lock:
                self.results = None
                self.reults_qr = None
            return result
        world_locations = []
        with self.mode_lock:
            self.mode = PerceptionMode.APPLE
        if goal.order == VisionScan.Goal.APPLE_SCAN:
            for cons in local_results:
                valid, pose = self.consToPoseEstimate(cons)
                if valid:
                    world_locations.append(pose)
        else:
            cons = local_results[0]
            valid, pose = self.consToPoseEstimate(cons)
            if valid:
                world_locations.append(pose)
        
        if len(world_locations) == 0:
            goal_handle.abort()
            result = VisionScan.Result()
            result.status = "Cloud Scan failure!"
            with self.results_lock:
                self.results = None
                self.reults_qr = None
            return result

        feedback_msg = VisionScan.Feedback()
        feedback_msg.success = True
        if goal.order == VisionScan.Goal.APPLE_SCAN:
            feedback_msg.apples = world_locations
        else:
            feedback_msg.qr_pose = world_locations[0]
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = VisionScan.Result()
        result.status = "Success"

        self.get_logger().info(f'Goal succeeded!')
        return result


    def consToPoseEstimate(self, cons: ConsensusStruct):
        if self.cx == None:
            return False, None
        x1 = int(min(cons.u1, cons.u2))
        y1 = int(min(cons.v1, cons.v2))
        x2 = int(max(cons.u1, cons.u2))
        y2 = int(max(cons.v1, cons.v2))
        roi = cons.depth_field[y1:y2, x1:x2]
        valid = np.isfinite(roi) & (roi > 10) & (roi < 65535) #Remove NaN and <10mm

        if np.count_nonzero(valid) == 0:
            return False, None

        z = roi[valid] / 1000.0
        
        v_coords, u_coords = np.indices(roi.shape)

        u = u_coords[valid] + x1
        v = v_coords[valid] + y1

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        points = np.vstack((x, y, z)).T
        centroid = np.mean(points, axis=0)

        centroid_pose = Pose()

        centroid_pose.position.x = centroid[0]
        centroid_pose.position.y = centroid[1]
        centroid_pose.position.z = centroid[2]

        transformedPose = tf2_geometry_msgs.do_transform_pose(centroid_pose, cons.tf)

        print(f'{transformedPose.position.x} {transformedPose.position.y} {transformedPose.position.z}')

        pose = PoseStamped()
        pose.pose = transformedPose
        
        pose.pose.orientation.x = 0.5
        pose.pose.orientation.y = -0.5
        pose.pose.orientation.z = 0.5
        pose.pose.orientation.w = -0.5
        pose.header.frame_id = "base_link"

        return True, pose
