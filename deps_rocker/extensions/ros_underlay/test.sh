#!/bin/bash
set -e

echo "Testing ros_underlay extension..."

# Check if build-underlay command exists
if ! command -v build-underlay &> /dev/null; then
    echo "ERROR: build-underlay command not found"
    exit 1
fi

# Check if the underlay directory was created
UNDERLAY_ROOT="${ROS_UNDERLAY_PATH:-/workspaces/ros_ws/underlay}"
if [ ! -d "$UNDERLAY_ROOT" ]; then
    echo "ERROR: $UNDERLAY_ROOT directory not found"
    exit 1
fi

# Test that the build-underlay command runs without error
build-underlay

# Check environment variables are set

if [[ ":$AMENT_PREFIX_PATH:" != *":$UNDERLAY_ROOT:"* ]]; then
    echo "ERROR: AMENT_PREFIX_PATH does not contain $UNDERLAY_ROOT"
    exit 1
fi

echo "ros_underlay extension test completed successfully!"
