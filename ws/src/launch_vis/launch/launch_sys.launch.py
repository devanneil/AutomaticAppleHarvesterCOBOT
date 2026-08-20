from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the other launch file
    pkg_name = 'launch_vis'
    launch_cam = os.path.join(
        get_package_share_directory(pkg_name),
        'launch',
        'launch_cam.launch.py'
    )
    launch_perception = os.path.join(
        get_package_share_directory(pkg_name),
        'launch',
        'launch_perception.launch.py'
    )
    launch_arm = os.path.join(
        get_package_share_directory(pkg_name),
        'launch',
        'launch_single_arm.launch.py'
    )

    # Include the other launch file
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_cam)
    )
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_perception)
    )

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_arm)
    )

    return LaunchDescription([
        cam_launch,
        perception_launch,
        arm_launch
    ])
