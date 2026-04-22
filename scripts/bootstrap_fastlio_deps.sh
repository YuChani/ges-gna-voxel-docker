#!/usr/bin/env bash
# bootstrap_fastlio_deps.sh
# Sets up catkin workspace structure and fetches dependencies for FAST-LIO.
# Called inside Docker container at /root/catkin_ws/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "${SCRIPT_DIR}")"

echo "=== Setting up catkin workspace ==="

# 1. Create src/ directory and symlink FAST_LIO as catkin package
mkdir -p "${WS_DIR}/src"
if [ ! -e "${WS_DIR}/src/FAST_LIO" ]; then
    ln -s "${WS_DIR}/FAST_LIO" "${WS_DIR}/src/FAST_LIO"
    echo "  -> Symlinked FAST_LIO into src/"
fi

# 2. Clone ikd-Tree if not already present
IKD_DIR="${WS_DIR}/FAST_LIO/include/ikd-Tree"
if [ ! -f "${IKD_DIR}/ikd_Tree.h" ]; then
    echo "  -> Cloning ikd-Tree..."
    rm -rf "${IKD_DIR}"
    git clone --branch fast_lio --depth 1 \
        https://github.com/hku-mars/ikd-Tree.git "${IKD_DIR}"
else
    echo "  -> ikd-Tree already present"
fi

# 3. Remove CATKIN_IGNORE if present
if [ -f "${WS_DIR}/FAST_LIO/CATKIN_IGNORE" ]; then
    rm "${WS_DIR}/FAST_LIO/CATKIN_IGNORE"
    echo "  -> Removed CATKIN_IGNORE"
fi

# 4. Install livox_ros_driver if not already installed
if ! rospack find livox_ros_driver &>/dev/null; then
    echo "  -> Installing livox_ros_driver..."
    apt-get update -qq && apt-get install -y --no-install-recommends ros-noetic-pcl-ros 2>/dev/null || true
    if [ ! -d "${WS_DIR}/src/livox_ros_driver" ]; then
        git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git \
            "${WS_DIR}/src/livox_ros_driver" || echo "  -> Warning: livox_ros_driver clone failed (non-fatal for velodyne/ouster configs)"
    fi
else
    echo "  -> livox_ros_driver already available"
fi

echo "=== Bootstrap complete ==="
