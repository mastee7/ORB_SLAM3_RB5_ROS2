#!/bin/bash
echo "Building ROS2 nodes"

# This script builds the ROS2 ORB_SLAM3 wrapper using colcon.
# Prerequisites:
#   1. ROS2 must be sourced (e.g. source /opt/ros/humble/setup.bash)
#   2. The main ORB_SLAM3 library must already be built (run ./build.sh first)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_PKG_DIR="${SCRIPT_DIR}/Examples/ROS2/ORB_SLAM3"

# Create a temporary colcon workspace
WORKSPACE="${SCRIPT_DIR}/ros2_ws"
mkdir -p "${WORKSPACE}/src"

# Symlink the ROS2 package into the workspace
ln -sfn "${ROS2_PKG_DIR}" "${WORKSPACE}/src/orb_slam3_ros2"

# Build with colcon
cd "${WORKSPACE}"
colcon build --packages-select orb_slam3_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "Build complete. To use, source the workspace:"
echo "  source ${WORKSPACE}/install/setup.bash"
