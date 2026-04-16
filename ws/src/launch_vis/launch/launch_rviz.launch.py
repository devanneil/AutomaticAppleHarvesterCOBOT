from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "gcr16_2000",
            package_name="robot_moveit_config"
        ).to_moveit_configs()
    )

    return LaunchDescription([

        # -------------------------
        # RViz only
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            )
        )
    ])