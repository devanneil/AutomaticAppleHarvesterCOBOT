import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray

from apple_interfaces.srv import ScanPose
from apple_interfaces.action import VisionScan

from dataclasses import dataclass
import time

@dataclass
class ScanPose:
    pose: PoseStamped
    ID: int
    claimed: bool
NEXT_ID = 0

RADIUS = VisionScan.Goal.RADIUS # pixels
TIME_BETWEEN_SCAN = 5 # seconds
MAX_RIGHT_SCAN = 0.7 # meters
SCAN_PLANE_DISTANCE = 1.966 # meters

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

        self.clear_subscription = self.create_subscription(
            Int32,
            '/scout1_cam/clear_pose',
            self.clear_callback,
            10,
            callback_group=self.group_1
        )

        self.pose_timer = self.create_timer(
            0.1,
            self.control_loop,
            callback_group=self.group_1
        )
        self.pose_publisher = self.create_publisher(
            PoseArray,
            "/scout1_cam/pose_visualizer",
            10
        )


        self.pose_list = []
        self.last_scan_time = 0
    
    def shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()    
        
    def check_tf_ready(self):
        if self.tf_ready:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                self.frame_id,
                rclpy.time.Time()     
            )

            self.tf_ready = True
            self.get_logger().info("TF ready: map -> scout_frame")

            self.tf_timer.cancel()

            # Example PoseStamped in base_link frame
            pose_in = PoseStamped()
            pose_in.header.frame_id = self.frame_id
            pose_in.header.stamp = self.get_clock().now().to_msg()
            pose_in.pose.position.x = 0.0
            pose_in.pose.position.y = SCAN_PLANE_DISTANCE
            pose_in.pose.position.z = 0.0
            pose_in.pose.orientation.x = 0.0
            pose_in.pose.orientation.y = 0.0
            pose_in.pose.orientation.z = 0.0
            pose_in.pose.orientation.w = 1.0

            self.create_scan_pose(pose_in)

        except TransformException:
            self.get_logger().debug("Waiting for TF tree...")
            
    def create_scan_pose(self, pose_in):
        global NEXT_ID
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",
                    self.frame_id,
                    rclpy.time.Time()     
                )

                pose_out = do_transform_pose(pose_in.pose, transform)

                pose_stamped_out = PoseStamped()
                pose_stamped_out.header.frame_id = 'map'
                pose_stamped_out.header.stamp = self.get_clock().now().to_msg()  
                pose_stamped_out.pose = pose_out

                scan_pose = ScanPose(pose_stamped_out, NEXT_ID, False)
                NEXT_ID += 1
                self.pose_list.append(scan_pose)
                return
            except TransformException:
                self.get_logger().debug("Waiting for TF tree...")
            finally:
                rate = self.create_rate(2, self.get_clock())
                rate.sleep()

    def control_loop(self):
        if not self.tf_ready:
            return False
        now = time.time()
        newScan = False
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = "map"
        for scan_pose in self.pose_list:
            scan_pose.pose.header.stamp = self.get_clock().now().to_msg()
            pa.poses.append(scan_pose.pose.pose)
        self.pose_publisher.publish(pa)

        if now - self.last_scan_time > TIME_BETWEEN_SCAN:
            left_scan = self.get_leftmost_scan()
            if left_scan is None:
                return
            left_pose = left_scan.pose
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    "map",
                    rclpy.time.Time()     
                )
            except TransformException:
                self.get_logger().debug("Waiting for TF tree...")
                return
            pose_out = do_transform_pose(left_pose.pose, transform)
            if pose_out.position.z < -MAX_RIGHT_SCAN: #Right -> -z
                newScan = True
            
        if newScan:
            self.get_logger().info("Scanning here!")
            self.last_scan_time = time.time()
            # Example PoseStamped in base_link frame
            pose_in = PoseStamped()
            pose_in.header.frame_id = self.frame_id
            pose_in.header.stamp = self.get_clock().now().to_msg()
            pose_in.pose.position.x = 0.0
            pose_in.pose.position.y = SCAN_PLANE_DISTANCE
            pose_in.pose.position.z = 0.0
            pose_in.pose.orientation.x = 0.0
            pose_in.pose.orientation.y = 0.0
            pose_in.pose.orientation.z = 0.0
            pose_in.pose.orientation.w = 1.0

            self.create_scan_pose(pose_in)

    def get_leftmost_scan(self):
        if len(self.pose_list) > 0:
            return max(
                self.pose_list,
                key=lambda scan: scan.pose.pose.position.y
            )
        else:
            return None
    def get_rightmost_scan(self):
        if len(self.pose_list) > 0:
            return min(
                self.pose_list,
                key=lambda scan: scan.pose.pose.position.y
            )
        else:
            return None
    def clear_callback(self, msg):
        for scan_pose in self.pose_list:
            if msg.data == scan_pose.ID:
                self.pose_list.remove(scan_pose)

    def image_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id,
                rclpy.time.Time()     
            )
        except TransformException:
            self.get_logger().debug("Waiting for TF tree...")
            return

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