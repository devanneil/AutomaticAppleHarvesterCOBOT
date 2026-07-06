from cam_drive_control.camera_driver import CameraDriver
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

import cv2
import threading
import time

node = None
def main(args=None):
    global node

    rclpy.init(args=args)

    node = CameraDriver()

    # Create CV Context
    cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(
        "Camera View",
        mouse_event_handler,
        node,
    )
    ros_thread = threading.Thread(target=node_thread)
    ros_thread.start()

    while rclpy.ok():
        if node is None:
            time.sleep(0.1)
            continue
        frame = None
        image_results = node.get_image_results()
        if image_results is not None:
            annotated_img = image_results.plot()
            if annotated_img is not None:
                cv2.imshow("Camera View", annotated_img)
        else:
            frame = node.get_image_raw()
            if frame is not None:
                cv2.imshow("Camera View", frame)

        key = cv2.waitKey(10) & 0xFF

        if key == 27:
            rclpy.shutdown()
            break

        if cv2.getWindowProperty("Camera View", cv2.WND_PROP_VISIBLE) < 1:
            rclpy.shutdown()
            break

    cv2.destroyAllWindows()
    ros_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

def node_thread():
    global node
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()

# Mouse callback function
def mouse_event_handler(event, x, y, flags, node):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    node.handle_click(x, y)
