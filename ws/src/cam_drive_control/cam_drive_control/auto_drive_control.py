import threading
import time

import cv2
import rclpy
from rclpy.executors import MultiThreadedExecutor

from cam_drive_control.camera_driver import CameraDriver, PerceptionMode


def ros_thread(node):
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = CameraDriver()
    node.headless = True

    # OpenCV GUI should remain on the main thread.
    cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)

    thread = threading.Thread(
        target=ros_thread,
        args=(node,),
        daemon=True,
    )
    thread.start()

    try:
        while rclpy.ok():
            # -----------------------------
            # Retrieve current display state
            # -----------------------------
            image_results, qr_results = node.get_image_results()

            if node.mode == PerceptionMode.APPLE and image_results is not None:
                annotated_img = image_results.plot()

                for cons in node.selected_results:
                    cv2.rectangle(
                        annotated_img,
                        (int(cons.u1), int(cons.v1)),
                        (int(cons.u2), int(cons.v2)),
                        (0, 255, 0),
                        3,
                    )

                cv2.circle(
                    annotated_img,
                    node.center_coordinates,
                    node.scan_radius,
                    (0, 255, 0),
                    3,
                )

                cv2.imshow("Camera View", annotated_img)

            elif node.mode == PerceptionMode.QR and qr_results is not None:
                image = node.get_image_raw()

                if image is not None:
                    cons = node.results_qr

                    cv2.rectangle(
                        image,
                        (int(cons.u1), int(cons.v1)),
                        (int(cons.u2), int(cons.v2)),
                        (0, 255, 0),
                        3,
                    )

                    cv2.imshow("Camera View", image)

            else:
                frame = node.get_image_raw()

                if frame is not None:
                    cv2.imshow("Camera View", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                rclpy.shutdown()
                break

            if cv2.getWindowProperty(
                "Camera View",
                cv2.WND_PROP_VISIBLE,
            ) < 1:
                rclpy.shutdown()
                break

            # Don't burn CPU if there is no new GUI work.
            time.sleep(0.005)

    finally:
        if rclpy.ok():
            rclpy.shutdown()

        thread.join(timeout=2.0)

        node.destroy_node()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

