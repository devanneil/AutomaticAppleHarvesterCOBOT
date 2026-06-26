from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

import subprocess
import time
import os


# -----------------------------
# Wait for robot
# -----------------------------
def wait_for_network(context):

    robot_ip = LaunchConfiguration("robot_ip").perform(context)

    print(f"[Launch] Waiting for robot at {robot_ip}...")

    while True:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "2", robot_ip],
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            print(f"[Launch] Robot {robot_ip} reachable.")
            break

        time.sleep(2)

    time.sleep(3)

def wait_for_robot(context):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformListener
    from rclpy.duration import Duration
    import time

    print("[Launch] Waiting for joint_states + TF readiness...")

    rclpy.init()
    node = Node("robot_readiness_checker")

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    joint_state_ok = False
    last_joint_time = None

    def joint_cb(msg):
        nonlocal joint_state_ok, last_joint_time

        if len(msg.position) > 0:
            joint_state_ok = True
            last_joint_time = node.get_clock().now()

    node.create_subscription(
        JointState,
        "/joint_states",
        joint_cb,
        10
    )

    start = time.time()
    timeout = 30.0

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        if time.time() - start > timeout:
            raise RuntimeError("TF/joint_states readiness timeout")

        # 1. Wait for joint_states first (MOST IMPORTANT)
        if not joint_state_ok:
            continue

        # 2. Wait for TF tree to have ANY usable transform
        try:
            if tf_buffer.can_transform(
                "base_link",
                "link_6",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            ):
                print("[Launch] TF is ready.")
                break
        except Exception:
            continue

    node.destroy_node()
    rclpy.shutdown()

    print("[Launch] Robot ready.")
# -----------------------------
# Launch entry point
# -----------------------------
def generate_launch_description():

    # MoveIt config (SINGLE ARM)
    moveit_config = (
        MoveItConfigsBuilder(
            "gcr16_2000",
            package_name="robot_moveit_config"
        ).to_moveit_configs()
    )

    sys_config = os.path.join(
        get_package_share_directory("launch_vis"),
        "config",
        "params.yaml"
    )

    models_share = get_package_share_directory("robot_model")

    urdf_file = os.path.join(
        models_share,
        "urdf",
        "bin_filler.urdf"
    )

    with open(urdf_file, "r") as f:
        robot_description = f.read()
    ld = LaunchDescription()

    # -----------------------------
    # Arguments (MUST BE ADDED)
    # -----------------------------
    ld.add_action(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "arm_num",
            default_value="1"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "arm_dof",
            default_value="6"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "arm_domain",
            default_value="1"
        )
    )

    # -----------------------------
    # Wait for robot connection
    # -----------------------------
    ld.add_action(OpaqueFunction(function=wait_for_network))

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            )
        )
    )

    # -----------------------------
    # Hardware Drivers (SINGLE ARM)
    # -----------------------------
    ld.add_action(
        Node(
            package="duco_ros_driver",
            executable="DucoDriver",
            parameters=[{
                "arm_num": LaunchConfiguration("arm_num"),
                "server_host_1": LaunchConfiguration("robot_ip"),
                "arm_domain": LaunchConfiguration("arm_domain")
            }]
        )
    )

    ld.add_action(
        Node(
            package="duco_ros_driver",
            executable="DucoRobotStatus",
            parameters=[{
                "arm_num": LaunchConfiguration("arm_num"),
                "arm_dof": LaunchConfiguration("arm_dof"),
                "server_host_1": LaunchConfiguration("robot_ip"),
                "arm_domain": LaunchConfiguration("arm_domain")
            }]
        )
    )

    ld.add_action(
        Node(
            package="duco_ros_driver",
            executable="DucoTrajectoryAction",
            parameters=[{
                "arm_num": LaunchConfiguration("arm_num"),
                "server_host_1": LaunchConfiguration("robot_ip"),
                "arm_domain": LaunchConfiguration("arm_domain")
            }]
        )
    )

    ld.add_action(
        Node(
            package="duco_ros_driver",
            executable="DucoRobotControl",
            parameters=[{
                "arm_num": LaunchConfiguration("arm_num"),
                "arm_dof": LaunchConfiguration("arm_dof"),
                "server_host_1": LaunchConfiguration("robot_ip"),
                "arm_domain": LaunchConfiguration("arm_domain")
            }]
        )
    )

    # Second wait gate
    #ld.add_action(OpaqueFunction(function=wait_for_robot))
    # -----------------------------
    # Move Group (Planner)
    # -----------------------------
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            )
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="bin_manager",
            parameters=[{
                    "robot_description": robot_description
                }]
        )
    )

    ld.add_action(
        Node(
            package="bin_manager",
            executable="bin_manager",
            parameters=[sys_config],
        )
    )

    return ld