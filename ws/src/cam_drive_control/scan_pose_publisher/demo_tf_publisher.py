#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_srvs.srv import Trigger
import tf2_ros
import math
import time
import threading
import copy

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_map_tf_publisher')
        self.target_pose_lock = threading.Lock()
        # Create a TransformBroadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.group_1 = ReentrantCallbackGroup()

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback, callback_group=self.group_1)
        self.srv = self.create_service(Trigger, 'move_robot', self.move_service_callback, callback_group=self.group_1)

        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()

    def shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def broadcast_timer_callback(self):
        step = 0.1
        with self.target_pose_lock:
            error = self.target_pose.pose.position.y - self.current_pose.pose.position.y

        dy = max(min(error, step), -step)

        self.current_pose.pose.position.y += dy

        #self.get_logger().info(f"DX value: {dx}")

        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'       # Parent frame
        t.child_frame_id = 'base_link'   # Child frame

        # Example: moving in a circle
        t.transform.translation.x = 0.0
        t.transform.translation.y = self.current_pose.pose.position.y
        t.transform.translation.z = 0.0

        # No rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

    def move_service_callback(self, request, response):
        with self.target_pose_lock:
            self.target_pose = copy.deepcopy(self.current_pose)
            self.target_pose.pose.position.y += 1.3589/4
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFPublisher()

    # MultiThreadedExecutor with 8 worker threads
    executor = MultiThreadedExecutor(num_threads=4)

    # Add node to executor
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once()
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
