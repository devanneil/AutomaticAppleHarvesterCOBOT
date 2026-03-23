#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/humble/setup.bash"
if [-f "/home/ws/install/setup.bash"]; then
    source "/home/ws/install/setup.bash"
fi
export DISPLAY=host.docker.internal:0


exec "$@"