#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, JointState, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
import numpy as np
import cv2
import torch
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive
import matplotlib.pyplot as plt

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
        self.rgbdCamera = None # Visual camera utility
        self.window_name = "Camera"
        self.confidence_threshold = 0.7
        #==================VARIABLES===========================
        self.results = None # Model annotations
        self.selected_box = None # Clicked annotation
        #==================TOPICS==============================
        # Create a subscriber to the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            Image,
            f'/arm1_cam/{camera_sn}/color/image_raw', 
            self.image_callback,
            10
        )

        self.camera_info_subscrition = self.create_subscription(
            CameraInfo,
            f'/arm1_cam/{camera_sn}/color/camera_info',
            self.info_callback,
            10
        )

        # Sub to depth field from camera, different dimensions from visual
        self.depth_subscription = self.create_subscription(
            PointCloud2,
            f'/arm1_cam/{camera_sn}/depth/points',
            self.point_callback,
            10
        )
        #===================TF Interface=======================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
    
    # Collect camera info and instantiate camera
    def info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.rgbdCamera = RGBCamera(msg, self.tf_buffer)
            self.get_logger().info("Captured camera_info")

            # stop subscription after first message
            self.destroy_subscription(self.camera_info_subscrition)
    
    #Set latest cloud to current cloud
    def point_callback(self, msg: PointCloud2):
        if self.rgbdCamera is None:
            return
        self.rgbdCamera.updateCloud(msg)
    
    # Mouse callback function
    def mouse_event_handler(self, event, x, y, flags, param):
        """
        Handles mouse events and prints coordinates.
        Draws markers for visual feedback.
        """
        img = param  # The image passed from setMouseCallback

        if self.rgbdCamera is None or self.rgbdCamera.processing is True:
            return None

        if event != cv2.EVENT_LBUTTONDOWN:
            return None

        if self.results is None:
            return None

        result = self.results[0]

        for box in result.boxes:

            # Bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # Confidence
            confidence = float(box.conf[0])

            # Class ID
            class_id = int(box.cls[0])

            # Class name
            class_name = self.model.names[class_id]

            # Check if click is inside box
            if x1 <= x <= x2 and y1 <= y <= y2:

                # Positive consensus
                if confidence >= self.confidence_threshold:

                    self.selected_box = (x1, y1, x2, y2)
                    points = self.extract_points_from_box(self.selected_box)

                    ax = plt.axes(projection='3d')
                    ax.scatter(points[:,0], points[:,1], points[:,2], s=1)
                    plt.show()

        return None

    # Find 3D points from box consensus  
    def extract_points_from_box(self, box):

        filtered_points = self.rgbdCamera.find_rect_consensus(box)

        return filtered_points

    # Handle keyboard input for esc key and wasd control
    def control_loop(self):
        key = cv2.waitKey(1) & 0xFF 
        escape = False
        if key != 255:
            escape = self.parse_key(key)

        # Check if window was closed
        if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1 or escape is True:
            self.get_logger().info("Camera window closed. Shutting down node.")
            cv2.destroyAllWindows()
            self.complete = True

    # Process cv keycodes 
    def parse_key(self, key):
        if key == 27:
            return True # Escape key
        # Get pose
        pose = self.get_current_pose()
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

class RGBCamera:
    """Camera model for parsing real world coordinates from image and depth field"""
    def __init__(self, camera_info, tf_buffer):
        #Constants
        self.width = 640
        self.height = 480
        self.fx = camera_info.k[0]
        self.fy = camera_info.k[4]
        self.cx = camera_info.k[2]
        self.cy = camera_info.k[5]
        self.cam_frame_id = camera_info.header.frame_id
        self.depth_frame_id = None
        #Variables
        self.cloud_msg = None
        self.xyz_cloud = None
        self.frame_id = None
        #Control flag
        self.processing = False
        #ROS interface
        self.tf_buffer = tf_buffer
    
    def updateCloud(self, cloud_msg):
        if self.processing is True:
            return # Ignore changes while thinking
        self.cloud_msg = cloud_msg

        if self.depth_frame_id is None:
            self.depth_frame_id = cloud_msg.header.frame_id
        
        pts = np.array(
            [(p[0], p[1], p[2])
            for p in point_cloud2.read_points(
                cloud_msg,
                field_names=("x","y","z"),
                skip_nans=False
            )],
            dtype=np.float32
        )

        self.xyz_cloud = pts.reshape(480,640,3)
    
    # Normalized ray in real space
    def pixel_to_ray(self, u, v):
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        d = np.array([x, y, 1.0])
        dNorm = d / np.linalg.norm(d)
        if self.depth_frame_id is None:
            return dNorm # No depth frame, skip here
        transform = self.tf_buffer.lookup_transform(
            self.depth_frame_id,
            self.cam_frame_id,
            rclpy.time.Time()
        )
        q = transform.transform.rotation
        T = quat_to_rot_matrix([q.x, q.y, q.z, q.w])
        R = T[:3, :3]
        ray_depth = R @ dNorm
        ray_depth = ray_depth / np.linalg.norm(ray_depth)
        return ray_depth

    # Finds closes point in cloud to image coordinate
    def find_best_match(self, u, v):

        ray = self.pixel_to_ray(u, v)

        best_point = None
        best_dist = float("inf")

        for col in self.xyz_cloud:
            for p in col:
                # skip invalid points
                if np.linalg.norm(p) < 1e-6:
                    continue

                # perpendicular distance to ray
                dist = np.linalg.norm(np.cross(p, ray))

                # optional: ensure point is in front of camera
                if np.dot(p, ray) <= 0:
                    continue

                if dist < best_dist:
                    best_dist = dist
                    best_point = p

        return best_point, best_dist

    # Finds real world coordinates for rectangle region from consensus image
    def find_rect_consensus(self, box, buffer = 0.1, frame_id=None):
        self.processing = True
        x1, y1, x2, y2 = map(int, box)

        c1, _ = self.find_best_match(x1, y1)
        c2, _ = self.find_best_match(x2, y2)

        if c1 is None or c2 is None:
            return []

        x_min = min(c1[0], c2[0]) - buffer
        y_min = min(c1[1], c2[1]) - buffer

        x_max = max(c1[0], c2[0]) + buffer
        y_max = max(c1[1], c2[1]) + buffer

        xs = self.xyz_cloud[:, :, 0]
        ys = self.xyz_cloud[:, :, 1]
        zs = self.xyz_cloud[:, :, 2]

        mask = (
            (xs > x_min) & (xs < x_max) &
            (ys > y_min) & (ys < y_max)
        )

        points = self.xyz_cloud[mask]

        self.processing = False
        return np.array(points)

def quat_to_rot_matrix(q):
    x, y, z, w = q

    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])

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
