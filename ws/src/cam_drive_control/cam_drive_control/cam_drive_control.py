#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, JointState
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

package_name = 'cam_drive_control'
class CameraDriver(Node):
    """A minimal ROS2 Python node that logs a message periodically."""

    def __init__(self):
        super().__init__('camera_driver')
        self.complete = False
        self.step = 0.01
        # Create a subscriber to the /camera/image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/arm1_cam/color/image_raw',  # Change to your topic name
            self.listener_callback,
            10
        )

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
        
        # Create CV Context
        self.window_name = "Camera"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # Create Control Thread Timer
        self.control_timer = self.create_timer(0.01, self.control_loop)

        # TF Position Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        # Moveit move client
        self.move_client = ActionClient(
            self,
            MoveGroup,
            "/move_action"
        )

    def get_current_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",   # base frame
                "link_6",      # end effector
                rclpy.time.Time()
            )

            pose = PoseStamped()

            pose.header.frame_id = "base_link"
            # FIX: assign values individually
            pose.pose.position.x = (
                transform.transform.translation.x
            )
            pose.pose.position.y = (
                transform.transform.translation.y
            )
            pose.pose.position.z = (
                transform.transform.translation.z
            )

            pose.pose.orientation.x = (
                transform.transform.rotation.x
            )
            pose.pose.orientation.y = (
                transform.transform.rotation.y
            )
            pose.pose.orientation.z = (
                transform.transform.rotation.z
            )
            pose.pose.orientation.w = (
                transform.transform.rotation.w
            )

            return pose

        except Exception as e:
            self.get_logger().warn(str(e))
            return None

    def send_pose_goal(self, target_pose, link_name):

        goal_msg = MoveGroup.Goal()

        # REQUIRED
        goal_msg.request.group_name = "arm_1"
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        goal_msg.planning_options.plan_only = True
        # Build constraints

        constraints = Constraints()

        # -------------------------
        # Position Constraint
        # -------------------------

        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = link_name

        # Small tolerance box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX

        box.dimensions = [
            0.05,
            0.05,
            0.001,
        ]

        position_constraint.constraint_region.primitives.append(
            box
        )

        position_constraint.constraint_region.primitive_poses.append(
            target_pose.pose
        )

        position_constraint.weight = 1.0

        constraints.position_constraints.append(
            position_constraint
        )

        # -------------------------
        # Orientation Constraint
        # -------------------------

        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = link_name

        orientation_constraint.orientation = (
            target_pose.pose.orientation
        )

        orientation_constraint.absolute_x_axis_tolerance = 0.2
        orientation_constraint.absolute_y_axis_tolerance = 0.2
        orientation_constraint.absolute_z_axis_tolerance = 0.2

        orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(
            orientation_constraint
        )
        # -----------------------
        # Join Constraint
        # ----------------------

        joint_names = self.current_joint_state.name
        joint_positions = self.current_joint_state.position

        for name, pos in zip(joint_names, joint_positions):

            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = pos
            joint_constraint.tolerance_above = 0.1
            joint_constraint.tolerance_below = 0.1
            joint_constraint.weight = 1.0
            
            constraints.joint_constraints.append(
                joint_constraint
            )

        goal_msg.request.goal_constraints.append(
            constraints
        )

        self.move_client.wait_for_server()

        goal_future = self.move_client.send_goal_async(goal_msg)

    def change_model(self, params):
        for param in params:
            if param.name == 'model_name':
                model_name = param.value

                if model_name == "None":
                    self.get_logger().warn("Model set to None, disabling inference.")
                    self.model = None
                    self.model_name = "None"
                    return SetParametersResult(successful=True)

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
    def image_process(self, image_bgr):
        if self.model == None:
            return image_bgr

        # Run YOLO inference
        results = self.model(image_bgr, verbose=False)

        # Render results on image
        annotated_img = results[0].plot()
        return annotated_img
        
        
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def listener_callback(self, msg: Image):
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
