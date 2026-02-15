#!/bin/bash

# Allow local connections to X server for GUI applications (Rviz)
xhost +local:root > /dev/null

# Container name and image name
CONTAINER_NAME="vins_container"
IMAGE_NAME="vins_humble"

# Run the container
# --net=host: shares network stack with host (important for ROS 2)
# --privileged: gives access to connected hardware (cameras/IMU)
# --volume: mounts the current source directory into the workspace
docker run -it \
    --net=host \
    --privileged \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PWD":/root/colcon_ws/src/VINS-Fusion-ROS2-humble-arm \
    --name $CONTAINER_NAME \
    $IMAGE_NAME \
    bash
