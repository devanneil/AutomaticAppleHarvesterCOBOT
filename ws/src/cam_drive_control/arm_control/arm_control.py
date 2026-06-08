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
        #===================Variables==========================
        self.last_apple_pose = None
        #===================TF Interface=======================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #===================Topics=============================
        self.apple_location_pub = self.create_subscription(
            PoseStamped,
            '/cam_drive_control/apple_location',
            self.apple_callback,
            10
        )
        #===================Moveit Interface===================
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action"
        )
        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik"
        )
        #===================Joint State Buffer=================
        self.latest_joint_state = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )
        #====================Timer worker=====================
        self.control_timer = self.create_timer(1.0, self.execute_pick_sequence)

        self.get_logger().info("Arm Controller node ready!")

    def apple_callback(self, msg: PoseStamped):
        self.last_apple_pose = msg

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def IKSolver(self, target_pose):
        req = GetPositionIK.Request()

        req.ik_request.group_name = "arm_1"
        req.ik_request.robot_state.is_diff = True
        req.ik_request.robot_state.joint_state = self.latest_joint_state
        req.ik_request.ik_link_name = "tool_frame"
        req.ik_request.pose_stamped = target_pose
        req.ik_request.timeout.sec = 2

        future = self.ik_client.call_async(req)

        start = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if future.done():
                response = future.result()
                return response.solution.joint_state

            # timeout safeguard
            if (self.get_clock().now() - start).nanoseconds * 1e-9 > 3.0:
                self.get_logger().error("IK timeout")
                return None

    def move_to_pose(self, target_pose: PoseStamped, timeout=20.0):
        if target_pose is None:
            self.get_logger().warn("Target pose is None")
            return False

        if self.latest_joint_state is None:
            self.get_logger().warn("No joint state received yet")
            return False

        # -----------------------------
        # Wait for MoveIt server
        # -----------------------------
        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveGroup action server not available")
            return False
        # Create constraints for move goal
        constraints = Constraints()
        ik_state = self.IKSolver(target_pose)
        if ik_state is None:
            return False
        for name, pos in zip(
            ik_state.name,
            ik_state.position
        ):
            jc = JointConstraint()

            jc.joint_name = name
            jc.position = pos

            self.get_logger.info(f"{name}: {np.degrees(pos)}")

            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001

            jc.weight = 1.0

            constraints.joint_constraints.append(jc)
        # # ===================Position Constraints===============
        # position_constraint = PositionConstraint()

        # position_constraint.header.frame_id = "base_link"
        # position_constraint.link_name = "tool_frame"

        # primitive = SolidPrimitive()
        # primitive.type = SolidPrimitive.BOX
        # primitive.dimensions = [0.005, 0.005, 0.005]        

        # box_pose = Pose()
        # box_pose.position.x = target_pose.pose.position.x
        # box_pose.position.y = target_pose.pose.position.y
        # box_pose.position.z = target_pose.pose.position.z

        # position_constraint.constraint_region.primitives.append(
        #     primitive
        # )

        # position_constraint.constraint_region.primitive_poses.append(
        #     box_pose
        # )

        # position_constraint.weight = 1.0

        # #====================Orientation Constraint=================
        # orientation_constraint = OrientationConstraint()

        # orientation_constraint.header.frame_id = "base_link"

        # orientation_constraint.link_name = "tool_frame"

        # orientation_constraint.orientation = (
        #     target_pose.pose.orientation
        # )

        # orientation_constraint.absolute_x_axis_tolerance = 0.1
        # orientation_constraint.absolute_y_axis_tolerance = 0.1
        # orientation_constraint.absolute_z_axis_tolerance = 0.1

        # orientation_constraint.weight = 1.0

        # # Combine constraints
        # constraints.position_constraints.append(
        #     position_constraint
        # )

        # constraints.orientation_constraints.append(
        #     orientation_constraint
        # )
        self.get_logger().info("Goal constraints built!")
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

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True  # IMPORTANT: actually execute

        # -----------------------------
        # Send goal (async)
        # -----------------------------
        send_future = self.move_group_client.send_goal_async(goal)

        # We will wait manually
        start_time = self.get_clock().now()

        goal_handle = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if send_future.done():
                goal_handle = send_future.result()
                break

            if (self.get_clock().now() - start_time).nanoseconds * 1e-9 > timeout:
                self.get_logger().error("Timed out waiting for MoveIt goal acceptance")
                return False

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected")
            return False

        # -----------------------------
        # Wait for result
        # -----------------------------
        result_future = goal_handle.get_result_async()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if result_future.done():
                break

            if (self.get_clock().now() - start_time).nanoseconds * 1e-9 > timeout:
                self.get_logger().error("Timed out waiting for MoveIt execution")
                return False

        response = result_future.result()

        if response is None or response.result is None:
            self.get_logger().error("Invalid MoveIt result")
            return False

        if response.result.error_code.val == 1:
            self.get_logger().info("Motion succeeded")
            return True

        self.get_logger().error(f"Motion failed with code: {response.result.error_code.val}")
        return False

    def get_current_pose(
            self,
            tool_frame="tool_frame",
            base_frame="base_link"):
        if self.tf_buffer is None:
            self.get_logger().warn("TF_buffer not connected!")
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

    def execute_pick_sequence(self):
        if self.tf_buffer is None:
            self.get_logger().warn("TF_buffer not connected!")
            return None

        if self.last_apple_pose is None:
            return None

        self.get_logger().info("Moving to pick apple!")
        target_pose = self.last_apple_pose
        self.last_apple_pose = None
        home = self.get_current_pose()

        approach = copy.deepcopy(target_pose)
        approach.pose.position.z += 0.01

        #self.move_to_pose(approach)

        self.move_to_pose(target_pose)

        #self.perform_action()

        #self.move_to_pose(approach)

        #self.move_to_pose(home)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()