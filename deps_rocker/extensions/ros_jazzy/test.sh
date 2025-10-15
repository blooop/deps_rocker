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

# Test building a package that depends on the underlay
if [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then
    echo "Testing workspace build with underlay dependencies..."

    # Create a test package in the main workspace
    mkdir -p src/test_package
    cp /tmp/test_package.xml src/test_package/package.xml
    cp /tmp/test_setup.py src/test_package/setup.py
    mkdir -p src/test_package/resource
    touch src/test_package/resource/test_package
    mkdir -p src/test_package/test_package
    touch src/test_package/test_package/__init__.py

    # Source environments and build
    source /opt/ros/jazzy/setup.bash
    source "${ROS_UNDERLAY_INSTALL}/setup.bash"

    # Test that we can find the dependency
    if ! ros2 pkg list | grep -q unique_identifier_msgs; then
        echo "ERROR: unique_identifier_msgs still not found after sourcing underlay"
        exit 1
    fi

    # Test building the main workspace
    echo "Building main workspace with underlay dependency..."
    if ! colcon build --packages-select test_package; then
        echo "ERROR: Failed to build test package that depends on underlay"
        exit 1
    fi
    echo "✓ Successfully built package depending on underlay"

    # Test that we can source the built workspace
    if [ -f install/setup.bash ]; then
        source install/setup.bash
        echo "✓ Successfully sourced built workspace"
    else
        echo "ERROR: Built workspace setup.bash not found"
        exit 1
    fi
fi

echo "ROS Jazzy extension test completed successfully!"
