#!/bin/bash
# Run the ORB-SLAM3 Monocular node in ROS2.
#
# Prerequisites:
#   1. ROS2 must be sourced (e.g. source /opt/ros/humble/setup.bash)
#   2. The colcon workspace must be sourced:
#        source ros2_ws/install/setup.bash
#
# This is equivalent to the ROS1 command:
#   rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples_old/Monocular/rb5_camera_0_dp.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ros2 run orb_slam3_ros2 Mono \
    "${SCRIPT_DIR}/Vocabulary/ORBvoc.txt" \
    "${SCRIPT_DIR}/Examples_old/Monocular/rb5_camera_0_dp.yaml"
