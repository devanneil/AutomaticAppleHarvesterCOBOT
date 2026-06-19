import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("launch_vis")

    # -----------------------------------
    # Camera Definitions
    # -----------------------------------
    cameras = [
        {
            "name": "arm1_cam",
            "sn": "GDS871PBAA7110621",
            "param_file": "arm1_cam.yaml",
        },
        # {
        #     "name": "scout1_cam",
        #     "sn": "GDS871PBAA7110753",
        #     "param_file": "scout1_cam.yaml",
        # },
    ]

    composable_nodes = []

    # -----------------------------------
    # Create composable camera nodes
    # -----------------------------------
    for cam in cameras:

        param_path = os.path.join(
            pkg_share,
            "config",
            cam["param_file"]
        )

        composable_nodes.append(
            ComposableNode(
                package="scepter_manager",
                plugin="ScepterManager",
                name=cam['name'],
                parameters=[
                    param_path,
                    {   
                        "camera_name": cam["name"],
                        "camera_sn": cam["sn"],
                    }
                ]
            )
        )

    # -----------------------------------
    # Container
    # -----------------------------------
    container = ComposableNodeContainer(
        name="vzense_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
        emulate_tty=True,
    )

    nodes = [container]

    # -----------------------------------
    # Static TFs
    # -----------------------------------
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.0", "0.0", "0.0",
                "0", "0", "-1.57079633",
                "camera_link",
                "arm1_cam_frame",
            ]
        )
    )

    return LaunchDescription(nodes)