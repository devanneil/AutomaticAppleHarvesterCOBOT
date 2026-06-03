#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, JointState, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from sensor_msgs_py import point_cloud2
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
    PlanningOptions
)
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
arm_controller = None
class CameraDriver(Node):
    """A ROS node that reads from the rgbd camera, runs a model, and fetches the points near the consensus with wasd pan control"""

    def __init__(self):
        super().__init__('camera_driver')
        global arm_controller
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
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.pick_active:
            self.get_logger().warn("Pick already running — ignoring click")
            return

        self.pending_click = (x, y)
    
    # Threaded picking process
    @timer
    def process_pick(self, x, y):

        if self.rgbdCamera is None or self.results is None:
            return

        result = self.results[0]

        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            if x1 <= x <= x2 and y1 <= y <= y2:

                if float(box.conf[0]) < self.confidence_threshold:
                    return

                self.selected_box = (x1, y1, x2, y2)

                points = self.extract_points_from_box(self.selected_box)

                if points is None or len(points) == 0:
                    return

                ax = plt.axes(projection='3d') 
                ax.scatter(points[:,0], points[:,1], points[:,2], s=1) 
                plt.show()

                centroid = np.mean(points, axis=0)
                print(centroid)

                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    self.rgbdCamera.depth_frame_id,
                    rclpy.time.Time()
                )

                pt = PointStamped()
                pt.header.frame_id = self.rgbdCamera.cam_frame_id
                pt.point.x = float(centroid[0])
                pt.point.y = float(centroid[1])
                pt.point.z = float(centroid[2])

                transformed = do_transform_point(pt, transform)

                target_point = np.array([
                    transformed.point.x,
                    transformed.point.y,
                    transformed.point.z
                ])

                print(target_point)

                target_pose = build_pose(*target_point[:3], 0.7071068, 0.0, 0.7071068, 0.0)

                arm_controller.execute_pick_sequence(target_pose)

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
        global arm_controller
        if arm_controller is not None and arm_controller.tf_buffer is not None:
            #===================Moveit Interface==================
            arm_controller.connect_tf(self.tf_buffer)
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
        
    def pixel_to_ray(self, u, v):

        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy

        ray_cam = np.array([x, y, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)

        transform = self.tf_buffer.lookup_transform(
            self.depth_frame_id,
            self.cam_frame_id,
            rclpy.time.Time()
        )

        q = transform.transform.rotation

        R = quat_to_rot_matrix(
            [q.x, q.y, q.z, q.w]
        )

        ray_depth = R @ ray_cam
        ray_depth /= np.linalg.norm(ray_depth)

        origin_depth = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        return ray_depth, origin_depth

    # Finds closes point in cloud to image coordinate
    @timer
    def find_best_match(self, u, v):

        if self.xyz_cloud is None:
            return None

        ray, origin = self.pixel_to_ray(u, v)

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
                if np.dot(p - origin, ray) <= 0:
                    continue

                if dist < best_dist:
                    best_dist = dist
                    best_point = p

        return best_point, best_dist

    # Finds real world coordinates for rectangle region from consensus image
    @timer
    def find_rect_consensus(self, box, buffer = 0.1, frame_id=None):
        if self.xyz_cloud is None:
            return None

        self.processing = True
        x1, y1, x2, y2 = map(int, box)

        c1, _ = self.find_best_match(x1, y1)
        c2, _ = self.find_best_match(x2, y2)

        if c1 is None or c2 is None:
            return None

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

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        #===================TF Interface=======================
        self.tf_buffer = None
        self.tf_listener = None
        #===================Moveit Interface===================
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action"
        )
        #===================Joint State Buffer=================
        self.latest_joint_state = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )
    def connect_tf(self, tf_buffer):
        self.tf_buffer = tf_buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def move_to_pose(self, target_pose):
        if target_pose is None:
            return
        if self.latest_joint_state is None:
            self.get_logger().warn("Empty Joint State Buffer")
            return
        # Wait for moveit server
        self.move_group_client.wait_for_server()

        # Create constraints for move goal
        constraints = Constraints()
        # ===================Position Constraints===============
        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "suction_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.005, 0.005, 0.005]        

        box_pose = Pose()
        box_pose.position.x = target_pose.pose.position.x
        box_pose.position.y = target_pose.pose.position.y
        box_pose.position.z = target_pose.pose.position.z

        position_constraint.constraint_region.primitives.append(
            primitive
        )

        position_constraint.constraint_region.primitive_poses.append(
            box_pose
        )

        position_constraint.weight = 1.0

        #====================Orientation Constraint=================
        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = "base_link"

        orientation_constraint.link_name = "suction_link"

        orientation_constraint.orientation = (
            target_pose.pose.orientation
        )

        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1

        orientation_constraint.weight = 1.0

        # Combine constraints
        constraints.position_constraints.append(
            position_constraint
        )

        constraints.orientation_constraints.append(
            orientation_constraint
        )

        # Motion Plan Request
        request = MotionPlanRequest()

        request.group_name = "arm_1"

        request.start_state.is_diff = True
        request.start_state.joint_state = self.latest_joint_state
        request.goal_constraints.append(
            constraints
        )

        request.planner_id = "RRTConnectkConfigDefault"

        request.num_planning_attempts = 10

        request.allowed_planning_time = 5.0

        # MoveGroup Goal
        goal = MoveGroup.Goal()

        # Only plan the move
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True

        goal.request = request

        future = self.move_group_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "MoveGroup goal rejected"
            )
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        response = result_future.result()

        if response is None:
            self.get_logger().error("No response from MoveGroup action")
            return

        result = response.result

        if result is None:
            self.get_logger().error("No MoveIt result inside response")
            return

        if result.error_code.val == 1:
            self.get_logger().info("Motion succeeded")

        pass

    def get_current_pose(
            self,
            tool_frame="suction_link",
            base_frame="base_link"):
        if self.tf_buffer is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                tool_frame,
                rclpy.time.Time()
            )

            pose = PoseStamped()

            pose.header.frame_id = base_frame
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z

            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w

            return pose

        except Exception as e:
            self.get_logger().error(
                f"Failed to get current pose: {e}"
            )
            return None

    def execute_pick_sequence(self, target_pose):

        home = self.get_current_pose()

        approach = copy.deepcopy(target_pose)
        approach.pose.position.z += 0.10

        self.move_to_pose(approach)

        #self.move_to_pose(target_pose)

        #self.perform_action()

        #self.move_to_pose(approach)

        #self.move_to_pose(home)

def main(args=None):
    global arm_controller
    rclpy.init(args=args)
    node = CameraDriver()
    arm_controller = ArmController()
    try:
        while not node.complete:
            rclpy.spin_once(node)  # Keep the node alive
            rclpy.spin_once(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
