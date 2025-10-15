#!/bin/bash
set -e

echo "Testing ROS Jazzy extension..."

# Test that ROS environment is available
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    exit 1
fi

# Test ROS environment variables
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS_DISTRO not set"
    exit 1
fi

if [ "$ROS_DISTRO" != "jazzy" ]; then
    echo "ERROR: Expected ROS_DISTRO=jazzy, got $ROS_DISTRO"
    exit 1
fi

# Test that workspace directories exist
if [ ! -d "$ROS_WORKSPACE_ROOT" ]; then
    echo "ERROR: ROS workspace root not found at $ROS_WORKSPACE_ROOT"
    exit 1
fi

# Test that colcon is available
if ! command -v colcon &> /dev/null; then
    echo "ERROR: colcon command not found"
    exit 1
fi

# Test that vcstool is available
if ! command -v vcs &> /dev/null; then
    echo "ERROR: vcs command not found"
    exit 1
fi

# Test that rosdep is available
if ! command -v rosdep &> /dev/null; then
    echo "ERROR: rosdep command not found"
    exit 1
fi

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Test underlay packages are available if they were built
if [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then
    echo "Underlay install found, sourcing..."
    source "${ROS_UNDERLAY_INSTALL}/setup.bash"

    # Test that underlay packages are available
    echo "Testing underlay package availability..."
    if ! ros2 pkg list | grep -q unique_identifier_msgs; then
        echo "ERROR: unique_identifier_msgs not found in package list"
        exit 1
    fi
    echo "✓ Underlay package unique_identifier_msgs found"
else
    echo "No underlay install found - this is expected if no depends.repos was provided"
fi

# Test basic ROS functionality
echo "Testing basic ROS functionality..."
ros2 --help > /dev/null
echo "✓ ros2 help working"

echo "ROS Jazzy extension test completed successfully!"