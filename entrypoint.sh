#!/bin/bash
# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0
set -e

# Setup ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Build and source the workspace if it's mounted
if [ -d "/ws/src/garmi_description" ]; then
    colcon build
    source install/setup.bash
fi

exec "$@"
