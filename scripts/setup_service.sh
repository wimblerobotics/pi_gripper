#!/bin/bash
# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0
# Install, enable, and start the pi-gripper systemd service.
# Called by setup_robot.py post_build hook (run as sudo).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="${SCRIPT_DIR}/../systemd/pi-gripper.service"
cp "${SERVICE_FILE}" /etc/systemd/system/pi-gripper.service
systemctl daemon-reload
systemctl enable pi-gripper.service
systemctl start pi-gripper.service
echo "✓ pi-gripper service installed, enabled, and started."
