#!/bin/bash
set -e

ROS_DISTRO=${ROS_DISTRO:-jazzy}

if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    exit 1
fi

ros2 --version

if ! env | grep -q "ROS_DISTRO=$ROS_DISTRO"; then
    echo "ERROR: ROS_DISTRO environment variable not set to $ROS_DISTRO"
    exit 1
fi

echo "ros_generic extension test for ROS_DISTRO=$ROS_DISTRO completed successfully!"
