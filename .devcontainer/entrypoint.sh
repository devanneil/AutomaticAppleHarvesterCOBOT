#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/humble/setup.bash"
source "/home/ws/install/setup.bash"
export DISPLAY=host.docker.internal:0


exec "$@"