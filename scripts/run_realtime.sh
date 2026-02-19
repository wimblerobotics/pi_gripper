#!/bin/bash
# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0

# Wrapper script to run a ROS2 command with real-time scheduling priority and
# CPU affinity pinning.  Pass the ros2 command and arguments as parameters.
# Example: ./run_realtime.sh ros2 run pi_gripper pi_gripper_node

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace environment
WORKSPACE_DIR="/home/ros/sigyn_vision_ws"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# Set ROS_DOMAIN_ID if not already set
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Run the command with real-time priority (FIFO, priority 50) and CPU affinity
exec chrt -f 50 taskset -c 2 "$@"
