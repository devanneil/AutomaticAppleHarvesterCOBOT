#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class InteractiveTF(Node):

    def __init__(self):
        super().__init__('interactive_tf')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tty = open('/dev/tty', 'r')

        # Parameters
        self.parent_frame = "base_link"

        self.child_frame = "map"

        self.step = self.declare_parameter(
            'step',
            0.01
        ).value

        # Current pose
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.timer = self.create_timer(
            0.02,
            self.timer_callback
        )

    def timer_callback(self):

        self.handle_keyboard()

        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z

        # Identity rotation
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

    def handle_keyboard(self):

        # Non-blocking keyboard read
        if select.select([self.tty], [], [], 0)[0]:
            key = self.tty.read(1)

            # Arrow keys arrive as escape sequences:
            # ESC [ A = Up
            # ESC [ B = Down
            # ESC [ C = Right
            # ESC [ D = Left

            if key == '\x1b':

                if select.select([self.tty], [], [], 0)[0]:
                    key += self.tty.read(1)

                if select.select([self.tty], [], [], 0)[0]:
                    key += self.tty.read(1)

                if key == '\x1b[A':       # Up
                    self.y -= self.step

                elif key == '\x1b[B':     # Down
                    self.y += self.step

                # elif key == '\x1b[C':     # Right
                #     self.y -= self.step

                # elif key == '\x1b[D':     # Left
                #     self.y += self.step

            # elif key == 'w':
            #     self.z += self.step

            # elif key == 's':
            #     self.z -= self.step

            # elif key == 'q':
            #     self.get_logger().info('Shutting down.')
            #     rclpy.shutdown()

            # self.get_logger().info(
            #     f'Position: '
            #     f'x={self.x:.3f}, '
            #     f'y={self.y:.3f}, '
            #     f'z={self.z:.3f}'
            # )

def main():

    rclpy.init()

    node = InteractiveTF()

    old_settings = termios.tcgetattr(node.tty)

    try:
        tty.setcbreak(node.tty.fileno())

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        termios.tcsetattr(
            node.tty,
            termios.TCSADRAIN,
            old_settings
        )

        node.tty.close()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()