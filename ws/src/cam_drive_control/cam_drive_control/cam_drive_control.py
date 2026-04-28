#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
import numpy as np
import cv2
import torch
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

package_name = 'cam_drive_control'
class CameraDriver(Node):
    """A minimal ROS2 Python node that logs a message periodically."""

    def __init__(self):
        super().__init__('camera_driver')
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
        results = self.model(image_bgr)

        # Render results on image
        annotated_img = results[0].plot()
        return annotated_img
        
        
        

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
            cv2.waitKey(1)

            # Check if window was closed
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.get_logger().info("Camera window closed. Shutting down node.")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)  # Keep the node alive
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
