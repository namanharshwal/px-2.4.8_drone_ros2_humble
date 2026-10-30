#!/bin/bash
# ==============================================================================
# Copyright (c) 2025-2026 Naman Harshwal. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
# Helper Script: Launch F450 Digital Twin Simulation + Hardware Bridge + Keyboard Teleop
# Author: Naman Harshwal
# ==============================================================================

source /opt/ros/humble/setup.bash
source /home/drone/robox_ws/install/setup.bash
export ROS_DOMAIN_ID=0

echo "========================================================================"
echo "    STARTING F450 DRONE HYBRID SIMULATION & HARDWARE TELEOPERATION"
echo "========================================================================"

# 1. Verify / Start MicroXRCEAgent if not already running
if ! pgrep -x "MicroXRCEAgent" > /dev/null; then
    echo "[INFO] Starting MicroXRCEAgent on /dev/ttyAMA0 @ 921600 baud..."
    sudo chmod 666 /dev/ttyAMA0
    /usr/local/bin/MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 &
    AGENT_PID=$!
    sleep 1
else
    echo "[INFO] MicroXRCEAgent is already running."
fi

# 2. Launch Robot State Publisher & 6-DOF Simulation Engine
echo "[INFO] Starting Robot State Publisher (3D F450 URDF) & Sim Engine..."
ros2 launch f450_brain f450_sim_teleop.launch.py &
LAUNCH_PID=$!
sleep 2

# 3. Clean exit handler
cleanup() {
    echo -e "\n[INFO] Shutting down simulation and nodes..."
    kill -SIGINT $LAUNCH_PID 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# 4. Start Interactive Keyboard Teleoperation in this terminal
echo "[INFO] Starting Keyboard Teleop Controller..."
ros2 run f450_brain keyboard_teleop

cleanup
