#!/bin/bash
set -e

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
# Micro-XRCE-DDS Agent Build & Install Script (Fast Build with ROS 2 Humble)
# Author: Naman Harshwal
# ==============================================================================
echo "=== Building Micro-XRCE-DDS Agent ==="

# Source ROS 2 Humble to leverage system-installed Fast-DDS
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

BUILD_DIR="$HOME/Micro-XRCE-DDS-Agent"
if [ ! -d "$BUILD_DIR" ]; then
    git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$BUILD_DIR"
fi

cd "$BUILD_DIR"
mkdir -p build && cd build
cmake -DUAGENT_USE_SYSTEM_FASTDDS=ON ..
make -j2
sudo make install
sudo ldconfig /usr/local/lib/

echo "=== Micro-XRCE-DDS Agent installed successfully! ==="
which MicroXRCEAgent
MicroXRCEAgent --help | head -n 5
