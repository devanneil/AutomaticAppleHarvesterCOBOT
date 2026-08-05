import rclpy
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray

from apple_interfaces.srv import ScanPoseRequest
from apple_interfaces.action import VisionScan

from dataclasses import dataclass
import time
import sys
import os
import threading
import numpy as np
import cv2
import random
from math import sqrt
import heapq
import multiprocessing
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from multiprocessing import Process, Queue
from queue import Empty, Full

from ultralytics import YOLO
from itertools import combinations, product


@dataclass
class ScanPose:
    pose: PoseStamped
    ID: int
    claimed: bool
NEXT_ID = 0

@dataclass
class RobotWorkspace:
    name: str
    arm_ID: int
    area: [float, float]
    origin: [float, float, float]
    critical: bool
robot_workspaces = []

GOAL_RADIUS = VisionScan.Goal.RADIUS # pixels
TIME_BETWEEN_SCAN = 5 # seconds
MAX_RIGHT_SCAN = 1.3462/2 # meters
SCAN_PLANE_DISTANCE = 1.859 # meters between scout camera and scanning plane
CAM_TO_WALL_DISTANCE = 0.1 # meters between arm camera and scanning plane
MAX_POSES_IN_WS = 10
CONFIDENCE_THRESHOLD = 0.85
RADIUS = int(0.25 * GOAL_RADIUS * 0.6)
MAX_DEPTH_SEARCH = 30
MAX_WIDTH_SEARCH = 50

models_share = get_package_share_directory("robot_model")
urdf_file = os.path.join(
    models_share,
    "urdf",
    "robot_workspace.urdf"
)

class ScoutCamera(Node):
    """A ROS node that reads from the scout camera and publishes scan positions for the arms to clear"""

    def __init__(self):
        super().__init__('scout_camera')

        self.declare_parameter('model_name', 'wsu-v9c.pt')

        self.apple_model_name = self.get_parameter('model_name').value

        if self.apple_model_name == "None":
            self.get_logger().warn("No model specified, returning raw image!")
            self.apple_model = None
        else:
            # Set to new model
            try:
                self.get_logger().info(f"Loading model: {self.apple_model_name}")

                model_path = os.path.join(
                    get_package_share_directory("cam_drive_control"),
                    'models',
                    self.apple_model_name
                )

                model = YOLO(model_path)

                self.apple_model = model

                self.get_logger().info("YOLO model loaded successfully.")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to load YOLO model: {e}"
                )

        self.search_request_queue = Queue(maxsize=1)
        self.search_result_queue = Queue(maxsize=1)

        self.search_process = Process(
            target=search_process_main,
            args=(
                self.search_request_queue,
                self.search_result_queue
            ),
            daemon=True
        )

        self.search_process.start()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_ready = False
        self.tf_timer = self.create_timer(0.2, self.check_tf_ready)

        self.control_group = MutuallyExclusiveCallbackGroup()
        self.camera_group = MutuallyExclusiveCallbackGroup()

        self.camera_sn = "GDS871PBAA7110753"
        self.frame_id = None
        # Create a subscriber to the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            Image,
            f'/scout1_cam/{self.camera_sn}/color/image_raw', 
            self.image_callback,
            10,
            callback_group=self.camera_group
        )

        # Create a subscriber to the /camera/image_raw topic
        self.camera_subscription = self.create_subscription(
            CameraInfo,
            f'/scout1_cam/{self.camera_sn}/color/camera_info',
            self.cam_info_callback,
            10,
            callback_group=self.camera_group
        )


        self.clear_subscription = self.create_subscription(
            UInt32,
            '/scout1_cam/clear_pose',
            self.clear_callback,
            10,
            callback_group=self.camera_group
        )

        self.pose_timer = self.create_timer(
            0.1,
            self.control_loop,
            callback_group=self.control_group
        )
        self.results_timer = self.create_timer(
            0.1,
            self.check_search_results,
            callback_group=self.control_group
        )
        self.pose_publisher = self.create_publisher(
            PoseArray,
            "/scout1_cam/pose_visualizer",
            10
        )

        self.scan_pose_service = self.create_service(
            ScanPoseRequest,
            "/scout1_cam/scan_pose_request",
            self.service_callback,
            callback_group=self.control_group
        )


        self.pose_list = []
        self.pose_list_lock = threading.Lock()
        self.last_scan_time = Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type)
        self.last_scan_tf = None
        self.image_lock = threading.Lock()
        self.latest_image_raw = None
        self.viewport_lock = threading.Lock()
        self.viewport_image = None
        self.scan_busy = False

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
    
    def shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()    
        
    def check_tf_ready(self):
        if self.frame_id is None:
            rate = self.create_rate(2, self.get_clock())
            rate.sleep()
            return
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

        except TransformException:
            self.get_logger().debug("Waiting for TF tree...")
            
    def convert_image(self, msg):
        try:
            if msg.encoding == "rgb8":
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img_array = img_array.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)

            elif msg.encoding == "bgr8":
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                return img_array.reshape((msg.height, msg.width, 3))

            elif msg.encoding == "mono8":
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                return img_array.reshape((msg.height, msg.width))

            elif msg.encoding == "16UC1":
                img_array = np.frombuffer(msg.data, dtype=np.uint16)
                return img_array.reshape((msg.height, msg.width))

            else:
                self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
                return None

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return None

    def create_scan_pose(self, pose_in):
        global NEXT_ID

        pose_out = do_transform_pose(pose_in.pose, self.last_scan_tf)

        pose_stamped_out = PoseStamped()
        pose_stamped_out.header.frame_id = 'map'
        pose_stamped_out.header.stamp = self.get_clock().now().to_msg()  
        pose_stamped_out.pose = pose_out
        pose_stamped_out.pose.position.z += 0.4024
        pose_stamped_out.pose.position.y += 0.3032

        scan_pose = ScanPose(pose_stamped_out, NEXT_ID, False)
        NEXT_ID += 1
        with self.pose_list_lock:
            self.pose_list.append(scan_pose)
        return

    def control_loop(self):
        try:
            while rclpy.ok():
                if self.cx is None:
                    self.get_logger().warn("Waiting for Camera intrinsics", throttle_duration_sec=5.0)
                    rate = self.create_rate(2, self.get_clock())
                    rate.sleep()
                elif self.latest_image_raw is None:
                    self.get_logger().warn("Waiting for Camera image", throttle_duration_sec=5.0)
                    rate = self.create_rate(2, self.get_clock())
                    rate.sleep()
                else:
                    break
            if not self.tf_ready:
                return False
            now = self.get_clock().now()
            newScan = False
            pa = PoseArray()
            pa.header.stamp = self.get_clock().now().to_msg()
            pa.header.frame_id = "map"

            state_critical = False
            pose_ws_count = 0
            for scan_pose in self.pose_list:
                scan_pose.pose.header.stamp = self.get_clock().now().to_msg()
                pa.poses.append(scan_pose.pose.pose)
                in_ws, critical = self.pose_in_ws(scan_pose.pose)
                if critical:
                    state_critical = True
                if in_ws:
                    pose_ws_count += 1  
            self.pose_publisher.publish(pa)  

            if not self.scan_busy:
                if state_critical:
                    self.get_logger().warn("Do not move here, critical poses to be cleared!", throttle_duration_sec=5.0)
                else:
                    if pose_ws_count < MAX_POSES_IN_WS: 
                        self.get_logger().warn("Move forward by one pane!", throttle_duration_sec=5.0)
                    else:
                        self.get_logger().warn("Too many scans to be cleared, don't move yet!", throttle_duration_sec=5.0)
            
                if (now.nanoseconds - self.last_scan_time.nanoseconds)/(10**9) > TIME_BETWEEN_SCAN:
                    left_scan = self.get_leftmost_scan()
                    if left_scan is None:
                        newScan = True
                    else:
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
                        if pose_out.position.y > MAX_RIGHT_SCAN: #Right -> +y
                            newScan = True

                if newScan:
                    with self.image_lock:
                        local_image = self.latest_image_raw
                    if local_image is None:
                        return
                    self.get_logger().info("Scanning here!")
                    self.scan_busy = True
                    results = self.apple_model(local_image, verbose=False)

                    self.process_results(results)
                    self.last_scan_time = rclpy.time.Time()
                    while rclpy.ok():
                        try:
                            self.last_scan_tf = self.tf_buffer.lookup_transform(
                                        "map",
                                        self.frame_id,
                                        self.last_scan_time  
                                    )
                            break
                        except TransformException as e:
                            self.get_logger().warn(
                                f"TF lookup failed: "
                                f"target='map', "
                                f"source='{self.frame_id}', "
                                f"stamp={self.last_scan_time.to_msg().sec}.{self.last_scan_time.to_msg().nanosec}: "
                                f"{e}"
                            )
                            rate = self.create_rate(2, self.get_clock())
                            rate.sleep()

                    self.get_logger().info("Scan Created")
                # Example PoseStamped in base_link frame
                # pose_in = PoseStamped()
                # pose_in.header.frame_id = self.frame_id
                # pose_in.header.stamp = self.last_scan_time.to_msg()
                # pose_in.pose.position.x = 0.0
                # pose_in.pose.position.y = 0.0
                # pose_in.pose.position.z = SCAN_PLANE_DISTANCE
                # pose_in.pose.orientation.x = 0.0
                # pose_in.pose.orientation.y = 0.0
                # pose_in.pose.orientation.z = 0.0
                # pose_in.pose.orientation.w = 1.0

                # self.create_scan_pose(pose_in)
        except Exception as e:
            self.get_logger().error(e)

    def get_leftmost_scan(self):
        with self.pose_list_lock:
            pose_list = self.pose_list
        if len(pose_list) > 0:
            return max(
                pose_list,
                key=lambda scan: scan.pose.pose.position.y
            )
        else:
            return None
    def get_rightmost_scan(self, pose_list=None):
        if pose_list is None:
            with self.pose_list_lock:
                pose_list = self.pose_list
        if len(pose_list) > 0:
            return min(
                pose_list,
                key=lambda scan: scan.pose.pose.position.y
            )
        else:
            return None
    def clear_callback(self, msg):
        for scan_pose in self.pose_list:
            if msg.data == scan_pose.ID:
                with self.pose_list_lock:
                    self.pose_list.remove(scan_pose)
                self.get_logger().info(f"Removed pose: {scan_pose.ID}")

    def image_callback(self, msg):
        with self.image_lock:
            self.latest_image_raw = self.convert_image(msg)    

    def cam_info_callback(self, msg):
        if self.cx is not None:
            return
        self.frame_id = msg.header.frame_id
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info("Successfully collected camera info!")
        
    def pose_in_ws(self, pose: PoseStamped, arm_id=None):
        global robot_workspaces
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                pose.header.frame_id,
                rclpy.time.Time()     
            )
        except TransformException:
            self.get_logger().debug("Waiting for TF tree...")
            return False, False
        pose_out = do_transform_pose(pose.pose, transform)

        x_base = pose_out.position.x
        y_base = pose_out.position.y
        z_base = pose_out.position.z

        if arm_id is None:
            valid_ws = robot_workspaces
        else:
            valid_ws = [ws for ws in robot_workspaces if ws.arm_ID == arm_id]
        for ws in valid_ws:
            y = y_base - ws.origin[1] + ws.area[0] / 2
            z = z_base - ws.origin[2] + ws.area[1] / 2

            if (
                0 <= y < ws.area[0]
                and 0 <= z < ws.area[1]
            ):
                return True, ws.critical

        return False, False

    def process_results(self, results):
        if results is None:
            return
        local_results = results[0]
        mask = local_results.boxes.conf > CONFIDENCE_THRESHOLD
        filtered_boxes = local_results.boxes[mask]
        if len(filtered_boxes) == 0:
            return
        local_results.boxes = filtered_boxes
        # Minimum covering circles for boxes
        xyxy = filtered_boxes.xyxy

        rectangles = [tuple(box.tolist()) for box in filtered_boxes.xyxy[:64]]

        try:
            self.search_request_queue.put_nowait(rectangles)
        except Full:
            self.get_logger().debug(
                "Search worker busy; skipping search"
            )

        with self.viewport_lock:
            self.viewport_image = local_results.plot()

    def check_search_results(self):
        try:
            result = self.search_result_queue.get_nowait()
        except Empty:
            return

        if isinstance(result, Exception):
            self.get_logger().error(
                f"Search process failed: {result}"
            )
            return

        if not result:
            self.get_logger().warn(
                "Search failed to find a solution"
            )
            return

        with self.viewport_lock:
            for circle in result:
                cv2.circle(self.viewport_image, (int(circle[0]), int(circle[1])), circle[2], (0,255,0),3)
            
            # x_max, y_max = self.viewport_image.shape[:2]
            # x1 = int(max(min(circle[0] for circle in result) - RADIUS - 40, 0))
            # y1 = int(max(min(circle[1] for circle in result) - RADIUS - 40, 0))
            # x2 = int(min(max(circle[0] for circle in result) + RADIUS + 40, x_max))
            # y2 = int(min(max(circle[1] for circle in result) + RADIUS + 40, y_max))


            # self.viewport_image = self.viewport_image[y1:y2, x1:x2]
        self.get_logger().info("Processing scan results")
        for circle in result:
            u, v, _ = circle
            z = SCAN_PLANE_DISTANCE
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy

            pose_in = PoseStamped()
            pose_in.header.frame_id = self.frame_id
            pose_in.header.stamp = self.last_scan_time.to_msg()
            pose_in.pose.position.x = -x
            pose_in.pose.position.y = -y
            pose_in.pose.position.z = SCAN_PLANE_DISTANCE
            pose_in.pose.orientation.x = 0.0
            pose_in.pose.orientation.y = 0.0
            pose_in.pose.orientation.z = 0.0
            pose_in.pose.orientation.w = 1.0

            self.create_scan_pose(pose_in)
        self.scan_busy = False
        self.get_logger().info("Scan results available")

    def service_callback(self, request, response):
        valid_poses = []
        self.get_logger().info(f"{len(self.pose_list)} scan poses remain!")
        for scan_pose in self.pose_list:
            in_ws, critical = self.pose_in_ws(scan_pose.pose, request.arm_num)
            if critical:
                response.header = scan_pose.pose.header
                response.pose = scan_pose.pose
                response.id = scan_pose.ID
                response.success = True
                return response
            if in_ws:
                valid_poses.append(scan_pose)
        result = self.get_rightmost_scan(valid_poses)
        if result is not None:
            response.header = result.pose.header
            response.pose = result.pose
            response.id = result.ID
            response.success = True
            return response
        
        response.success = False
        response.id = 0
        return response
        
            




def search_process_main(request_queue, result_queue):
    while True:
        rectangles = request_queue.get()

        if rectangles is None:
            break

        try:
            result = search_worker_main(rectangles)
            result_queue.put(result)
        except Exception as e:
            result_queue.put(e)

def search_worker_main(rectangles):
    coverage_mask = complete_coverage_mask(rectangles)

    # ------------------------------------------------------------
    # Generate candidate circles.
    #
    # Deduplicate based on coverage, since two circles producing
    # the exact same coverage are equivalent to the search.
    # ------------------------------------------------------------

    candidates = []
    candidate_coverages = set()

    for rectangle_pair in combinations(rectangles, 2):
        circles = circles_from_rectangle(
            rectangle_pair[0],
            rectangle_pair[1],
            RADIUS
        )

        for circle in circles:
            if circle is None:
                continue

            coverage = compute_coverage(circle, rectangles)

            if coverage == 0:
                continue

            if coverage in candidate_coverages:
                continue

            candidate_coverages.add(coverage)
            candidates.append((circle, coverage))

    for rectangle in rectangles:
        for point in rectangle_corners(rectangle):
            circle = (point[0], point[1], RADIUS)
            coverage = compute_coverage(circle, rectangles)

            if coverage == 0:
                continue

            if coverage in candidate_coverages:
                continue

            candidate_coverages.add(coverage)
            candidates.append((circle, coverage))

    print(f"Generated {len(candidates)} candidates")

    # ------------------------------------------------------------
    # Search states
    # ------------------------------------------------------------

    coverage_states = {
        0: []
    }

    current_states = {
        0: []
    }

    for depth in range(MAX_DEPTH_SEARCH):

        print(f"Search at depth {depth}")

        next_states = {}

        for coverage, state in current_states.items():

            # ----------------------------------------------------
            # Rank candidates relative to THIS state.
            # ----------------------------------------------------

            ranked_candidates = []

            for circle, candidate_coverage in candidates:

                new_coverage = coverage | candidate_coverage

                if new_coverage == coverage:
                    continue

                score = heuristic_method(new_coverage)

                ranked_candidates.append(
                    (score, circle, candidate_coverage)
                )

            # Best candidates first
            ranked_candidates.sort(
                key=lambda candidate: candidate[0],
                reverse=True
            )

            # ----------------------------------------------------
            # Only expand the best MAX_WIDTH_SEARCH candidates.
            # ----------------------------------------------------

            for score, circle, candidate_coverage in ranked_candidates[
                :MAX_WIDTH_SEARCH
            ]:

                new_coverage = coverage | candidate_coverage

                new_state = state + [circle]

                # We already found this coverage with fewer
                # circles, so this state cannot improve anything.
                if new_coverage in coverage_states:
                    if len(new_state) >= len(
                        coverage_states[new_coverage]
                    ):
                        continue

                coverage_states[new_coverage] = new_state
                next_states[new_coverage] = new_state

                if new_coverage == coverage_mask:
                    print(
                        f"Found solution at depth {depth + 1}"
                    )
                    return new_state

        current_states = next_states

        if not current_states:
            print("Search exhausted")
            break

    print("Failed to find solution!")
    return []

def heuristic_method(coverage):
    return coverage.bit_count()

def complete_coverage_mask(rectangles):
    count = min(len(rectangles), 64)
    return (1 << count) - 1

def is_in_circle(circle, rectangle):
    cx, cy, r = circle
    corners = rectangle_corners(rectangle)

    r_squared = r * r

    return any(
        (x - cx) ** 2 + (y - cy) ** 2 <= r_squared
        for x, y in corners
    )

def rectangle_corners(rectangle):
    x1, y1, x2, y2 = rectangle

    x1, x2 = min(x1, x2), max(x1, x2)
    y1, y2 = min(y1, y2), max(y1, y2)

    return [
        (x1, y1),
        (x1, y2),
        (x2, y1),
        (x2, y2)
    ]

def circles_from_rectangle(r1, r2, r):
    corners1 = rectangle_corners(r1)
    corners2 = rectangle_corners(r2)

    candidates = []

    for p1, p2 in product(corners1, corners2):
        circle1, circle2 = circles_from_p1p2r(p1, p2, r)

        if circle1 is not None:
            candidates.append(circle1)

        if circle2 is not None:
            candidates.append(circle2)

    return candidates

def circles_from_p1p2r(p1, p2, r):
    'Following explanation at http://mathforum.org/library/drmath/view/53027.html'
    (x1, y1), (x2, y2) = p1, p2
    if p1 == p2:
        #raise ValueError('coincident points gives infinite number of Circles')
        return None, None
    # delta x, delta y between points
    dx, dy = x2 - x1, y2 - y1
    # dist between points
    q = sqrt(dx**2 + dy**2)
    if q > 2.0*r:
        #raise ValueError('separation of points > diameter')
        return None, None
    # halfway point
    x3, y3 = (x1+x2)/2, (y1+y2)/2
    # distance along the mirror line
    d = sqrt(r**2-(q/2)**2)
    # One answer
    c1 = (x3 - d*dy/q,
            y3 + d*dx/q,
            abs(r))
    # The other answer
    c2 = (x3 + d*dy/q,
            y3 - d*dx/q,
            abs(r))
    return c1, c2

def compute_coverage(circle, rectangles):
    coverage = 0

    for i, rectangle in enumerate(rectangles[:64]):
        if is_in_circle(circle, rectangle):
            coverage |= 1 << i

    return coverage


def main(args=None):
    global robot_workspaces
    rclpy.init(args=args)
    try:
        with open(urdf_file, "r") as f:
            urdf_string = f.read()
        root = ET.fromstring(urdf_string)

        # Example: find all <custom_data> tags anywhere in the URDF
        for custom_tag in root.findall(".//workspace_area"):
            name = custom_tag.get("name")  # attribute
            ws_type = custom_tag.get("type")  
            #print(f"Workspace tag found: name={name}, type={ws_type}")
            bounding_box = custom_tag.find(".//rectangle").get("size")
            #print(f"Workspace area: {bounding_box}")
            bounding_origin = custom_tag.find(".//origin").get("xyz")
            #print(f"Workspace origin: {bounding_origin}")
            if "arm_1" in name:
                robot_number = 1
            else:
                robot_number = 0
                print(f"Invalid workspace name! {name}")
                continue
            if ws_type == "critical":
                ws_critical = True
            else:
                ws_critical = False
            ws_area = list(map(float, bounding_box.split()))
            ws_origin = list(map(float, bounding_origin.split()))
            ws = RobotWorkspace(name, robot_number, ws_area, ws_origin, ws_critical)
            robot_workspaces.append(ws)

    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}")
        sys.exit(1)
    
    # Sort workspaces by criticality
    robot_workspaces = sorted(robot_workspaces, key=lambda ws: ws.critical, reverse=True)
    print(f"Found {len(robot_workspaces)} workspaces!")
    node = ScoutCamera()

    # MultiThreadedExecutor with 8 worker threads
    executor = MultiThreadedExecutor(num_threads=6)

    # Add node to executor
    executor.add_node(node)

    cv2.namedWindow("Scout Camera View", cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            executor.spin_once()
            if node.viewport_image is not None:
                cv2.imshow("Scout Camera View", node.viewport_image)

            key = cv2.waitKey(10) & 0xFF

            if key == 27:
                rclpy.shutdown()
                break

            if cv2.getWindowProperty("Scout Camera View", cv2.WND_PROP_VISIBLE) < 1:
                rclpy.shutdown()
                break
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()