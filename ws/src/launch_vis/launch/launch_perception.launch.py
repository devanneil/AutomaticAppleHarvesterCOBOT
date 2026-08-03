from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_mode = LaunchConfiguration('auto_mode')
    demo_tf = LaunchConfiguration('demo_tf')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_mode',
            default_value='false',
            description='Run mode: auto or manual'
        ),
        DeclareLaunchArgument(
            'demo_tf',
            default_value='true',
            description='Run demo tf publisher?'
        ),

        Node(
            package='cam_drive_control',
            executable='auto_drive_control',
            condition=IfCondition(auto_mode)
        ),

        Node(
            package='cam_drive_control',
            executable='user_drive_control',
            condition=UnlessCondition(auto_mode)
        ),

        Node(
            package='cam_drive_control',
            executable='demo_tf_publisher',
            condition=IfCondition(demo_tf)
        ),

        Node(
            package='cam_drive_control',
            executable='scout_camera_scanner'
        ),

        Node(
            package='suction_commander',
            executable='suction_commander'
        )
    ])