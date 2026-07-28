from cam_drive_control.camera_driver import CameraDriver, PerceptionMode
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

import cv2
import threading
import time
import numpy as np

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
        image_results, qr_results = node.get_image_results()
        if node.mode == PerceptionMode.APPLE and image_results is not None:
            # Apple case, image_results is ultralytics object
            annotated_img = image_results.plot()
            for cons in node.selected_results:
                cv2.rectangle(
                    annotated_img,
                    (int(cons.u1), int(cons.v1)),
                    (int(cons.u2), int(cons.v2)),
                    (0, 255, 0),
                    3
                )
            if annotated_img is not None:
                height, width = annotated_img.shape[:2]
                center_coordinates = (width // 2, height // 2)
                cv2.circle(annotated_img, center_coordinates, 300, (0,255,0),3)
                cv2.imshow("Camera View", annotated_img)
        # QR Case, image_results is tuple of (4,2)
        elif node.mode == PerceptionMode.QR and qr_results is not None:
            image = node.get_image_raw()
            if image is not None:
                cons = node.results_qr
                cv2.rectangle(
                    image,
                    (int(cons.u1), int(cons.v1)),
                    (int(cons.u2), int(cons.v2)),
                    (0, 255, 0),
                    3
                )
                cv2.imshow("Camera View", image)
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
    executor = MultiThreadedExecutor(num_threads=5)
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
