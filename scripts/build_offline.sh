#!/usr/bin/env bash
# build_offline.sh
# Builds FAST-LIO assuming all dependencies are already present.
# Called inside Docker container at /root/catkin_ws/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "${SCRIPT_DIR}")"

# Ensure src/ symlink exists
mkdir -p "${WS_DIR}/src"
if [ ! -e "${WS_DIR}/src/FAST_LIO" ]; then
    ln -s "${WS_DIR}/FAST_LIO" "${WS_DIR}/src/FAST_LIO"
fi

# Remove CATKIN_IGNORE if present
rm -f "${WS_DIR}/FAST_LIO/CATKIN_IGNORE"

echo "=== Building FAST-LIO (offline) ==="
cd "${WS_DIR}"
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
echo "=== Build complete ==="
