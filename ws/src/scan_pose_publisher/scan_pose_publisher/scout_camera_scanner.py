import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from apple_interfaces.srv import ScanPose
from apple_interfaces.action import VisionScan

RADIUS = VisionScan.Goal.RADIUS


class ScoutCamera(Node):
    """A ROS node that reads from the scout camera and publishes scan positions for the arms to clear"""

    def __init__(self):
        super().__init__('scout_camera')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_ready = False
        self.tf_timer = self.create_timer(0.2, self.check_tf_ready)

        self.group_1 = MutuallyExclusiveCallbackGroup()

        self.camera_sn = "GDS871PBAA7110753"
        self.frame_id = "scout_link"
        # Create a subscriber to the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            Image,
            f'/scout1_cam/{self.camera_sn}/color/image_raw', 
            self.image_callback,
            10,
            callback_group=self.group_1
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/scout1_cam/pose_visualizer",
            10
        )
    
    def shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()    
        
    def check_tf_ready(self):
        if self.tf_ready:
            return

        try:
            T_dummy_test = self.tf_buffer.lookup_transform(
                "dummy_link",
                self.frame_id,
                rclpy.time.Time()
            )

            self.tf_ready = True
            self.get_logger().info("TF ready: dummy_link -> scout_frame")

            self.tf_timer.cancel()

        except TransformException:
            self.get_logger().debug("Waiting for TF tree...")

    def image_callback(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id

        pose.pose.position.z = 0.5

        self.pose_publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = ScoutCamera()

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

if __name__ == "__main__":
    main()