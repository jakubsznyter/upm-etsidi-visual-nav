#!/bin/bash

source /opt/ros/jazzy/setup.bash

source $HOME/robot_ws/hardware_host/install/setup.bash

# 3. Set Domain ID (Optional, but good for safety)
export ROS_DOMAIN_ID=0

# 4. Launch your robot
# We use 'exec' so the launch process replaces this script
exec ros2 launch $HOME/robot_ws/hardware_host/launch/robot_app.launch.py
