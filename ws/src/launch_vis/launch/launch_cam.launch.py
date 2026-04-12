from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # -----------------------------
    # Camera "dictionary"
    # -----------------------------
    cameras = [
        {"name": "arm1_cam", "ip": "192.168.1.101"},
        #{"name": "arm2_cam", "ip": "192.168.1.102"},
        #{"name": "arm3_cam", "ip": "192.168.1.103"},
        #{"name": "arm4_cam", "ip": "192.168.1.104"},
        {"name": "scout1_cam", "ip": "192.168.1.201"},
        #{"name": "scout2_cam", "ip": "192.168.1.202"},
    ]

    nodes = []

    for cam in cameras:
        nodes.append(
            Node(
                package="launch_vis",
                executable="scepter_cam_wrapper",
                arguments=[cam["name"], cam["ip"]]
            )
        )

    return LaunchDescription(nodes)