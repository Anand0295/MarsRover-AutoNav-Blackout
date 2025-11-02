# ROS Noetic + Gazebo classic base for MarsRover-AutoNav-Blackout
# Optimized for smaller image and reproducible OSS deployment

FROM ros:noetic-ros-core

# Set noninteractive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Install minimal dependencies: Gazebo, ROS control, Python tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-pip python3-numpy python3-scipy python3-yaml \
    git wget ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# Create workspace
ENV CATKIN_WS=/ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS

# Copy repository source
COPY . $CATKIN_WS/src/MarsRover-AutoNav-Blackout

# Python dependencies for planners and fusion
RUN pip3 install --no-cache-dir numpy scipy

# Build (noop if only Python code). Keep layer minimal.
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make || true

# Source ROS and set default entrypoint to Gazebo with world if provided
ENV ROS_DISTRO=noetic
SHELL ["/bin/bash", "-lc"]

# Example run command (documented in README):
# docker build -t autonov:oss .
# docker run --rm -it \ 
#   --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \ 
#   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#   autonov:oss bash

CMD ["bash"]
