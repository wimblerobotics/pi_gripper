#!/bin/bash
# SPDX-FileCopyrightText: 2026 Michael Wimble <mike@wimblerobotics.com>
# SPDX-License-Identifier: Apache-2.0

# Install and enable the pi-gripper systemd service.
# Run once during machine setup (requires sudo).
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="${SCRIPT_DIR}/../systemd/pi-gripper.service"
sudo cp "${SERVICE_FILE}" /etc/systemd/system/pi-gripper.service
sudo systemctl daemon-reload
sudo systemctl enable pi-gripper.service
sudo systemctl start pi-gripper.service
echo "pi-gripper service installed and started."
