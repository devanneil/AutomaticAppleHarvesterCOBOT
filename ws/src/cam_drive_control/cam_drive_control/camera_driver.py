import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from ament_index_python.packages import get_package_share_directory
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
from tf2_py import TransformException
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

        self.declare_parameter('model_name', 'wsu-v8m.pt')

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
        self.frame_lock = threading.Lock()
        self.latest_frame = None
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
        self.qr_message = None
        self.headless = False
        self.center_coordinates = (0, 0)
        self.scan_radius = int(VisionScan.Goal.RADIUS)
        self.goal_stamp = None
        self.latest_image_stamp = None
        self.image_condition = threading.Condition()
        #==================USER DEPENDENT VARIABLES===========
        self.cv_lock = threading.Lock()
        self.clicked_locations = list()
        #==================TOPICS==============================
        self.camera_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()
        # Create a subscriber to the /camera/image_raw topic
        self.vis_sub = Subscriber(self, Image, f'/arm1_cam/{camera_sn}/color/image_raw', callback_group=self.camera_group)
        self.depth_sub = Subscriber(self, Image, f'/arm1_cam/{camera_sn}/transformedDepth/image_raw', callback_group=self.camera_group)
        self.ts = ApproximateTimeSynchronizer(
            [self.vis_sub, self.depth_sub],
            2,
            0.03
        )
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
        self.control_thread_running = True
        self.control_thread = threading.Thread(
            target=self.control_loop,
            daemon=True
        )
        self.control_thread.start()

    def destroy_node(self):
        self.control_thread_running = False

        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)

        super().destroy_node()

    def control_loop(self):
        while self.control_thread_running:
            with self.image_condition:
                self.image_condition.wait(timeout=1.0)
            with self.frame_lock:
                if self.latest_frame is None:
                    continue

                image_msg, depth_msg = self.latest_frame
                self.latest_frame = None

            image_time = rclpy.time.Time.from_msg(image_msg.header.stamp)
            # now = self.get_clock().now()

            # age = (now - image_time).nanoseconds / 1e9

            # self.get_logger().info(f"Node control age: {age}")
            if self.goal_stamp is not None:
                if image_time < self.goal_stamp:
                    self.get_logger().debug(
                        "Discarding image captured before goal"
                    )
                    continue
            with self.image_lock:
                self.latest_image_raw = self.convert_image(image_msg)
                self.image_count_since_last_state += 1
            height, width = self.latest_image_raw.shape[:2]
            self.center_coordinates = (width // 2, height // 2)
            if self.mode == PerceptionMode.APPLE:
                self.detect_apples(self.latest_image_raw, depth_msg)

            if self.mode == PerceptionMode.QR:
                self.detect_qr(self.latest_image_raw, depth_msg)

            time.sleep(1)


    def image_callback(self, image_msg, depth_msg):
        image_time = rclpy.time.Time.from_msg(image_msg.header.stamp)
        # now = self.get_clock().now()

        # age = (now - image_time).nanoseconds / 1e9

        # self.get_logger().info(f"Image age: {age:.3f}s")
        with self.frame_lock:
            self.latest_frame = (image_msg, depth_msg)

        with self.image_condition:
            self.latest_image_stamp = image_msg.header.stamp
            self.image_condition.notify_all()


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

        depth_image = self.convert_image(depth_ros_msg)
        try:
            t_base_cam = self.tf_buffer.lookup_transform(
                "base_link",
                depth_ros_msg.header.frame_id,
                depth_ros_msg.header.stamp
            )
        except TransformException as e:
            self.get_logger().error(f"TF Error! {e}")
            return

        if not self.headless:
            for u, v in local_clicked:

                for box in self.results.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()


                    if x1 <= u <= x2 and y1 <= v <= y2:

                        if float(box.conf[0]) < self.confidence_threshold:
                            return

                        # self.get_logger().info(
                        #     f"Pixel Coordinates: u1:{x1}, v1{y1}, u1{x2}, v2{y2}"
                        # )
                        newCons = ConsensusStruct(x1, y1, x2, y2, box.conf[0],
                            depth_image, t_base_cam)
                        append = True
                        for cons in self.selected_results:
                            if consensusEqual(cons, newCons):
                                append = False
                        if append:
                            with self.selected_lock:
                                self.selected_results.append(newCons)
        else:
            with self.selected_lock:
                self.selected_results.clear()
            for box in self.results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                circle_coords_1 = (x1 - self.center_coordinates[0], y1 - self.center_coordinates[1])
                circle_coords_2 = (x2 - self.center_coordinates[0], y2 - self.center_coordinates[1])
                circle_coords_3 = (x1 - self.center_coordinates[0], y2 - self.center_coordinates[1])
                circle_coords_4 = (x2 - self.center_coordinates[0], y1 - self.center_coordinates[1])

                if ((circle_coords_1[0]**2 + circle_coords_1[1]**2 < self.scan_radius**2) 
                    or (circle_coords_2[0]**2 + circle_coords_2[1]**2 < self.scan_radius**2)
                    or (circle_coords_3[0]**2 + circle_coords_3[1]**2 < self.scan_radius**2)
                    or (circle_coords_4[0]**2 + circle_coords_4[1]**2 < self.scan_radius**2)):
                    if float(box.conf[0]) >= self.confidence_threshold:
                        newCons = ConsensusStruct(x1, y1, x2, y2, box.conf[0],
                            depth_image, t_base_cam)
                        with self.selected_lock:
                            self.selected_results.append(newCons)
        

    def detect_qr(self, image, depth_ros_msg):
        detections = self.qr_model.detect(image, False)
        if len(detections) != 0:
            best = max(detections, key=lambda d: d["confidence"])
            QR_text = self.qr_model.decode(image, best)
            if QR_text != self.qr_message:
                return
            x1, y1, x2, y2 = map(int, np.round(best["bbox_xyxy"]))

            padding = 40  # pixels

            height, width = image.shape[:2]

            x1 = max(0, x1 - padding)
            y1 = max(0, y1 - padding)
            x2 = min(width - 1, x2 + padding)
            y2 = min(height - 1, y2 + padding)
            try:
                t_base_cam = self.tf_buffer.lookup_transform(
                    "base_link",
                    depth_ros_msg.header.frame_id,
                    depth_ros_msg.header.stamp
                )
            except TransformException:
                self.get_logger().error("TF Error!")
                return
            newCons = ConsensusStruct(x1, y1, x2, y2, best["confidence"], 
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
        if goal_request.stamp.sec != 0 or goal_request.stamp.nanosec != 0:
            self.goal_stamp = rclpy.time.Time.from_msg(goal_request.stamp)
        else:
            self.goal_stamp = self.get_clock().now()
        now = self.get_clock().now()

        age = (now - self.goal_stamp).nanoseconds / 1e9

        self.get_logger().info(f"Goal age: {age:.3f}s")
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
            self.qr_message = goal_request.qr_message

                #self.get_logger().info("Changing to qr mode")

        with self.image_lock:
            self.image_count_since_last_state = 0
        with self.results_lock:
            self.results = None
            self.results_qr = None    
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT
    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        # Clear the pipeline to get latest frames
        with self.frame_lock:
            self.latest_frame = None

        with self.results_lock:
            self.results = None
            self.results_qr = None

        with self.selected_lock:
            self.selected_results.clear()
        # Wait until latest stamp matches current time
        goal_start = self.get_clock().now()

        with self.image_condition:
            while (
                self.latest_image_stamp is None or
                rclpy.time.Time.from_msg(self.latest_image_stamp) < goal_start
            ):
                self.image_condition.wait(timeout=1.0)
        
        timeout = 10.0  # seconds
        start_time = time.monotonic()

        while rclpy.ok():
            if time.monotonic() - start_time > timeout:
                self.get_logger().warn("Vision scan timed out.")

                goal_handle.abort()

                result = VisionScan.Result()
                result.status = "Vision scan timeout"

                with self.results_lock:
                    self.results = None
                    self.results_qr = None

                with self.selected_lock:
                    self.selected_results.clear()

                with self.mode_lock:
                    self.mode = PerceptionMode.APPLE

                return result
            if self.image_count_since_last_state > 20:
                goal_handle.abort()
                result = VisionScan.Result()
                if self.cx is not None:
                    result.status = "No consensus found!"
                else:
                    result.status = "Failed to init camera!"
                with self.results_lock:
                    self.results = None
                    self.results_qr = None
                self.mode = PerceptionMode.APPLE
                return result
            with self.results_lock:
                if self.mode == PerceptionMode.APPLE and self.results is not None and self.cx is not None:
                    local_results = list(self.selected_results)
                    self.selected_results.clear()
                    break
                if self.mode == PerceptionMode.QR and self.results_qr is not None and self.cx is not None:
                    local_results = list()
                    local_results.append(self.results_qr)
                    self.results_qr = None
                    break

            time.sleep(0.2)
        # Enable qr scanning and find consensus
        if len(local_results) == 0:
            goal_handle.abort()
            result = VisionScan.Result()
            result.status = "No selectable results"
            with self.results_lock:
                self.results = None
                self.results_qr = None
            return result
        world_locations = []
        with self.mode_lock:
            self.mode = PerceptionMode.APPLE
        if goal.order == VisionScan.Goal.APPLE_SCAN:
            for cons in local_results:
                valid, pose = self.houghPoseEstimate(cons)
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
                self.results_qr = None
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

        pose.pose.position.x += 0.01
        pose.pose.position.y -= 0.02
        pose.pose.position.z += 0.09
        
        pose.pose.orientation.x = 0.5
        pose.pose.orientation.y = -0.5
        pose.pose.orientation.z = 0.5
        pose.pose.orientation.w = -0.5
        pose.header.frame_id = "base_link"

        return True, pose


    def houghPoseEstimate(self, cons: ConsensusStruct):
        if self.cx is None:
            return False, None

        x1 = int(min(cons.u1, cons.u2))
        y1 = int(min(cons.v1, cons.v2))
        x2 = int(max(cons.u1, cons.u2))
        y2 = int(max(cons.v1, cons.v2))

        roi = cons.depth_field[y1:y2, x1:x2]

        valid = (
            np.isfinite(roi) &
            (roi > 10) &
            (roi < 65535)
        )
        
        if np.count_nonzero(valid) < 20:
            return False, None

        # Depth is mm -> meters
        z = roi[valid].astype(np.float32) / 1000.0

        v_coords, u_coords = np.indices(roi.shape)

        u = u_coords[valid] + x1
        v = v_coords[valid] + y1

        # Convert depth pixels -> camera coordinates
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        points = np.column_stack((x, y, z))

        # ------------------------------------------------------------
        # RANSAC sphere fitting
        # ------------------------------------------------------------

        # Expected apple radius in meters.
        # Adjust this to your actual apples.
        radius_min = 0.025
        radius_max = 0.060

        best_center = None
        best_radius = None
        best_inliers = None

        num_iterations = 500
        distance_threshold = 0.005  # 5 mm

        num_points = len(points)

        if num_points < 4:
            return False, None

        for _ in range(num_iterations):

            # Pick 4 random points
            indices = np.random.choice(num_points, 4, replace=False)
            sample = points[indices]

            # Solve sphere from 4 points
            p1 = sample[0]

            A = 2.0 * (sample[1:] - p1)
            b = (
                np.sum(sample[1:] ** 2, axis=1)
                - np.sum(p1 ** 2)
            )

            try:
                center = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                # Points were geometrically degenerate
                continue

            radius = np.linalg.norm(center - p1)

            # Reject spheres with unreasonable radius
            if radius < radius_min or radius > radius_max:
                continue

            # Calculate distance of every point from sphere surface
            distances = np.abs(
                np.linalg.norm(points - center, axis=1) - radius
            )

            inliers = distances < distance_threshold

            num_inliers = np.count_nonzero(inliers)

            if best_inliers is None or num_inliers > np.count_nonzero(best_inliers):
                best_center = center
                best_radius = radius
                best_inliers = inliers

        if best_center is None:
            return False, None

        # ------------------------------------------------------------
        # Refine sphere using all RANSAC inliers
        # ------------------------------------------------------------

        inlier_points = points[best_inliers]

        if len(inlier_points) < 10:
            return False, None

        # Linear least-squares sphere fit:
        #
        # x² + y² + z² =
        #     2cx*x + 2cy*y + 2cz*z + d
        #
        # radius² = cx² + cy² + cz² + d

        A = np.column_stack((
            2.0 * inlier_points[:, 0],
            2.0 * inlier_points[:, 1],
            2.0 * inlier_points[:, 2],
            np.ones(len(inlier_points))
        ))

        b = np.sum(inlier_points ** 2, axis=1)

        try:
            solution, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            return False, None

        center = solution[:3]
        d = solution[3]

        radius_squared = np.dot(center, center) + d

        if radius_squared <= 0:
            return False, None

        radius = np.sqrt(radius_squared)

        if radius < radius_min or radius > radius_max:
            return False, None

        # ------------------------------------------------------------
        # Lowest-X point on sphere
        # ------------------------------------------------------------

        surface_point = np.array([
            center[0] - radius,
            center[1],
            center[2]
        ])

        print(
            f"Sphere center: "
            f"{center[0]:.3f}, "
            f"{center[1]:.3f}, "
            f"{center[2]:.3f}"
        )

        print(f"Sphere radius: {radius:.3f}")

        print(
            f"Lowest-X surface point: "
            f"{surface_point[0]:.3f}, "
            f"{surface_point[1]:.3f}, "
            f"{surface_point[2]:.3f}"
        )

        # ------------------------------------------------------------
        # Transform camera/depth frame -> base_link
        # ------------------------------------------------------------

        surface_pose = Pose()

        surface_pose.position.x = float(surface_point[0])
        surface_pose.position.y = float(surface_point[1])
        surface_pose.position.z = float(surface_point[2])

        transformedPose = tf2_geometry_msgs.do_transform_pose(
            surface_pose,
            cons.tf
        )

        print(
            f"Base frame surface point: "
            f"{transformedPose.position.x:.3f} "
            f"{transformedPose.position.y:.3f} "
            f"{transformedPose.position.z:.3f}"
        )

        pose = PoseStamped()
        pose.pose = transformedPose
        pose.header.frame_id = "base_link"

        # Your existing tool offset
        pose.pose.position.x += 0.0
        pose.pose.position.y -= 0.03
        pose.pose.position.z += 0.04

        pose.pose.orientation.x = 0.5
        pose.pose.orientation.y = -0.5
        pose.pose.orientation.z = 0.5
        pose.pose.orientation.w = -0.5

        return True, pose