from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import subprocess
import time
import os

from ament_index_python.packages import get_package_share_directory

params_file = os.path.join(
    launch_package_path,
    "config",
    "duco_params.yaml"
)

def wait_for_robot(context):

    robot_ip = LaunchConfiguration("robot_ip").perform(context)

    print(f"Waiting for robot at {robot_ip}...")

    while True:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "5", robot_ip],
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            print(f"Robot {robot_ip} is reachable!")
            break

        print(result.stdout)
        print(result.stderr)
        time.sleep(5)
    time.sleep(20)


def generate_launch_description():

    # Declare launch argument
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.10",
        description="Robot IP address"
    )

    robot_ip = LaunchConfiguration("robot_ip")

    # Path to included launch
    other_launch_path = os.path.join(
        get_package_share_directory('duco_gcr16_2000_moveit_config'),
        'launch',
        'demo.launch.py'
    )

    return LaunchDescription([

        robot_ip_arg,

        # BLOCK until robot responds
        OpaqueFunction(
            function=wait_for_robot
        ),

        # Now launch robot stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_path),

            launch_arguments={
                "server_host_1": robot_ip
            }.items()
        ),

    ])