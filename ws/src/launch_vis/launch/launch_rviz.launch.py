from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "gcr16_2000",
            package_name="duco_gcr16_2000_moveit_config"
        ).to_moveit_configs()
    )

    return LaunchDescription([

        # -------------------------
        # Robot State Publisher
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            )
        ),

        # -------------------------
        # Move Group (for planning in RViz)
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            )
        ),

        # -------------------------
        # RViz only
        # -------------------------
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                str(moveit_config.package_path / "config/moveit.rviz")
            ],
            output="screen"
        )
    ])