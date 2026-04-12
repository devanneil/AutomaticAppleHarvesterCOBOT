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

import subprocess
import time


# -----------------------------
# Wait for robot
# -----------------------------
def wait_for_robot(context):

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


# -----------------------------
# Launch entry point
# -----------------------------
def generate_launch_description():

    # MoveIt config (SINGLE ARM)
    moveit_config = (
        MoveItConfigsBuilder(
            "gcr16_2000",
            package_name="duco_gcr16_2000_moveit_config"
        ).to_moveit_configs()
    )

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
    ld.add_action(OpaqueFunction(function=wait_for_robot))

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
    # Move Group (Planner)
    # -----------------------------
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
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

    return ld