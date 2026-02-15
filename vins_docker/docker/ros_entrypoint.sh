#!/bin/bash
set -e

# Source the main ROS 2 Humble setup
source "/opt/ros/humble/setup.bash"

# Source the VINS-Fusion workspace if it exists
if [ -f "/root/colcon_ws/install/setup.bash" ]; then
  source "/root/colcon_ws/install/setup.bash"
fi

exec "$@"
