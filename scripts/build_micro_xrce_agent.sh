#!/bin/bash
set -e

# ==============================================================================
# Micro-XRCE-DDS Agent Build & Install Script (Fast Build with ROS 2 Humble)
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
