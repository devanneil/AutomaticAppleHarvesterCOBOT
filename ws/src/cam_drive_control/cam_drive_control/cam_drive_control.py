#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, JointState, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from sensor_msgs_py import point_cloud2
from apple_interfaces.msg import AppleConsensus
import numpy as np
import cv2
import torch
import os
import copy
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    PositionIKRequest
)
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive
from duco_msg.msg import DucoRobotState
import matplotlib.pyplot as plt

#======================TIMER MODULE===========================
import time
import functools
import sys
import traceback

def timer(func):
    """
    A decorator that measures the execution time of a function.
    Works with any number of positional and keyword arguments.
    """
    @functools.wraps(func)  # Preserve original function metadata
    def wrapper_timer(*args, **kwargs):
        start_time = time.perf_counter()  # High-resolution timer
        try:
            result = func(*args, **kwargs)
        except Exception as e:
            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            print(f"[TIMER] Function '{func.__name__}' raised an exception after {elapsed_time:.6f} seconds.")
            traceback.print_exc(file=sys.stdout)
            raise  # Re-raise the exception after logging
        else:
            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            print(f"[TIMER] Function '{func.__name__}' executed in {elapsed_time:.6f} seconds.")
            return result
    return wrapper_timer


package_name = 'cam_drive_control'
class CameraDriver(Node):
    """A ROS node that reads from the rgbd camera, runs a model, and fetches the points near the consensus with wasd pan control"""

    def __init__(self):
        super().__init__('camera_driver')
        self.complete = False # Node exit flag
        #=======================Parameters=============================
        # Declare parameters with default values
        self.declare_parameter('model_name', 'None')
        self.add_on_set_parameters_callback(self.change_model)

        self.model = None
        self.model_name = self.get_parameter('model_name').value

        # Manually load initial model safely
        initial_param = Parameter(
            'model_name',
            Parameter.Type.STRING,
            self.model_name
        )

        self.change_model([initial_param])

        if self.model_name == "None":
            self.get_logger().warn("No model specified, returning raw image!")

        #==================CONSTANTS============================
        self.step = 0.01 # WASD step control
        camera_sn = "GDS871PBAA7110621" # Will come from hardware manager
        self.camera_info = None # Visual camera info
        self.window_name = "Camera"
        self.confidence_threshold = 0.7
        #==================VARIABLES===========================
        self.results = None # Model annotations
        self.selected_box = None # Clicked annotation
        self.pending_click = None
        self.pick_active = False
        #==================TOPICS==============================
        # Create a subscriber to the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            Image,
            f'/arm1_cam/{camera_sn}/color/image_raw', 
            self.image_callback,
            10
        )

        self.apple_consensus_pub = self.create_publisher(
            AppleConsensus,
            '/arm1/apple_consensus',
            10
        )
        #===================CV Interface======================
        # Create CV Context
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Camera", self.mouse_event_handler)

        # Create Control Thread Timer
        self.control_timer = self.create_timer(0.01, self.control_loop)

    # Dynamic model support
    def change_model(self, params):
        for param in params:
            if param.name == 'model_name':
                model_name = param.value

                # Set to no model
                if model_name == "None":
                    self.get_logger().warn("Model set to None, disabling inference.")
                    self.model = None # Memory safe
                    self.model_name = "None"
                    return SetParametersResult(successful=True)

                # Set to new model
                try:
                    self.get_logger().info(f"Loading model: {model_name}")

                    model_path = os.path.join(
                        get_package_share_directory(package_name),
                        'models',
                        model_name
                    )

                    model = YOLO(model_path)

                    self.model = model
                    self.model_name = model_name

                    self.get_logger().info("YOLO model loaded successfully.")

                    return SetParametersResult(successful=True)

                except Exception as e:
                    self.get_logger().error(
                        f"Failed to load YOLO model: {e}"
                    )

                    return SetParametersResult(successful=False)

        return SetParametersResult(successful=True)

    # Run image through model
    def image_process(self, image_bgr):
        if self.model == None:
            return image_bgr

        # Run YOLO inference
        self.results = self.model(image_bgr, verbose=False)

        # Render results on image
        annotated_img = self.results[0].plot()
        return annotated_img

    # ROS Callback for image
    def image_callback(self, msg: Image):
        try:
            # Convert ROS2 Image message to NumPy array
            img_array = np.frombuffer(msg.data, dtype=np.uint8)

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
                return

            image = self.image_process(img_bgr)
            # Display the image
            cv2.imshow(self.window_name, image)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
    
    # Mouse callback function
    def mouse_event_handler(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.pick_active:
            self.get_logger().warn("Pick already running — ignoring click")
            return

        self.pending_click = (x, y)
    
    # Threaded picking process
    @timer
    def process_pick(self, x, y):

        result = self.results[0]

        self.get_logger().info(
            f"orig_shape={result.orig_shape}"
        )

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()


            if x1 <= x <= x2 and y1 <= y <= y2:

                if float(box.conf[0]) < self.confidence_threshold:
                    return

                self.get_logger().info(
                    f"Pixel Coordinates: u1:{x1}, v1{y1}, u1{x2}, v2{y2}"
                )
                apple_message = AppleConsensus()
                apple_message.header.stamp = self.get_clock().now().to_msg()
                apple_message.header.frame_id = "arm1_cam_color_frame"  # or camera frame

                # Pixel coordinates of pick
                apple_message.u1 = int(x1)
                apple_message.v1 = int(y1)
                apple_message.u2 = int(x2)
                apple_message.v2 = int(y2)

                self.apple_consensus_pub.publish(apple_message)
                

                


    # Find 3D points from box consensus  
    def extract_points_from_box(self, box):

        filtered_points = self.rgbdCamera.find_rect_consensus(box)

        return filtered_points

    def shutdown(self):
        self.get_logger().info("Camera window closed. Shutting down node.")
        cv2.destroyAllWindows()
        self.complete = True
    # Handle keyboard input for esc key and wasd control
    def control_loop(self):
        key = cv2.waitKey(1) & 0xFF

        if key != 255:
            if self.parse_key(key):
                self.shutdown()
                return

        if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
            self.shutdown()
            return

        # -------------------------------
        # HANDLE PICK PIPELINE
        # -------------------------------
        if self.pending_click is not None and not self.pick_active:
            x, y = self.pending_click
            self.pending_click = None

            self.pick_active = True

            try:
                self.process_pick(x, y)
            except Exception as e:
                self.get_logger().error(f"Pick failed: {e}")
            finally:
                self.pick_active = False

    # Process cv keycodes 
    def parse_key(self, key):
        if key == 27:
            return True # Escape key
        # Get pose
        if key == 2490368 or key in (ord('w'), ord('W')): # Up
            pose.pose.position.z += self.step
        if key == 2621440 or key in (ord('s'), ord('S')): # Down
            pose.pose.position.z -= self.step
        if key == 2424832 or key in (ord('a'), ord('A')): # Left
            pose.pose.position.x += self.step
        if key == 2555904 or key in (ord('d'), ord('D')): # Right
            pose.pose.position.x -= self.step
        #DISABLED FOR TESTING / NOT CURRENTLY WORKING
        #self.send_pose_goal(pose, "link_6")

def build_pose(
        x,
        y,
        z,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=1.0,
        frame_id="base_link"):

    pose = PoseStamped()

    pose.header.frame_id = frame_id

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)

    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)

    return pose

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        while not node.complete:
            rclpy.spin_once(node)  # Keep the node alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
