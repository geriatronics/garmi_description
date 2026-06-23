# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0

FROM osrf/ros:jazzy-desktop

# Install additional packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-rqt-joint-trajectory-controller \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rqt-robot-steering \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

# Create workspace directory
RUN mkdir -p /ws/src
WORKDIR /ws

# Copy entrypoint script
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
