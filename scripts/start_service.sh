#!/bin/bash
# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0

# Start the pi_gripper ROS2 node.
source /opt/ros/jazzy/setup.bash
source /home/ros/sigyn_vision_ws/install/setup.bash
exec ros2 launch pi_gripper pi_gripper.launch.py
