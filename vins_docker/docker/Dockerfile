FROM ros:humble-perception

# Set non-interactive mode for apt and define workspace path
ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/root/colcon_ws
SHELL ["/bin/bash", "-c"]

# Install essential build tools and system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Manually compile Eigen 3.3.9 from source
# Ubuntu 22.04 defaults to 3.4.0, so manual build is required to meet specific version requirements
WORKDIR /tmp
RUN wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz && \
    tar -xzvf eigen-3.3.9.tar.gz && \
    cd eigen-3.3.9 && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) install && \
    cd /tmp && rm -rf eigen-3.3.9*

# Install Ceres Solver 2.1.0 via apt
# Ubuntu 22.04 repositories provide version 2.1.0 by default, avoiding long compilation times on ARM
RUN apt-get update && apt-get install -y \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep for automated ROS dependency management
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Setup workspace and copy source code
WORKDIR $COLCON_WS/src
COPY . ./VINS-Fusion-ROS2-humble-arm

# Install ROS package dependencies while skipping manually installed Eigen and Ceres
WORKDIR $COLCON_WS
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "libeigen3-dev libceres-dev" && \
    rm -rf /var/lib/apt/lists/*

# Build the project using colcon
# Explicitly point to the custom Eigen 3.3.9 path and limit parallel jobs to prevent OOM on Raspberry Pi
RUN source /opt/ros/humble/setup.bash && \
    colcon build \
    --symlink-install \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DEIGEN3_INCLUDE_DIR=/usr/local/include/eigen3 \
    --parallel-workers 2

# Configure entrypoint and default command
COPY ./docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
