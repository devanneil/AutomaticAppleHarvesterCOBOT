import os
import subprocess
from ament_index_python.packages import get_package_prefix


def main():
    import sys

    if len(sys.argv) < 3:
        print("Usage: scepter_cam_wrapper <camera_name> <camera_ip>")
        return

    camera_name = sys.argv[1]
    camera_ip = sys.argv[2]

    pkg_prefix = get_package_prefix("ScepterROS_MultiCameras")

    exe_path = os.path.join(
        pkg_prefix,
        "lib",
        "ScepterROS_MultiCameras",
        "scepter_multicameras"
    )

    os.execv(exe_path, [
        exe_path,
        camera_name,
        camera_ip
    ])


if __name__ == "__main__":
    main()