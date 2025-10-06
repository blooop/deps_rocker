#!/bin/bash
set -e

echo "Testing ros_underlay extension..."

# Check if build-underlay command exists
if ! command -v build-underlay &> /dev/null; then
    echo "ERROR: build-underlay command not found"
    exit 1
fi

# Check if /opt/ros_underlay directory was created
if [ ! -d "/opt/ros_underlay" ]; then
    echo "ERROR: /opt/ros_underlay directory not found"
    exit 1
fi

# Test that the build-underlay command runs without error
build-underlay

# Check environment variables are set
if [[ ":$AMENT_PREFIX_PATH:" != *":/opt/ros_underlay:"* ]]; then
    echo "ERROR: AMENT_PREFIX_PATH does not contain /opt/ros_underlay"
    exit 1
fi

echo "ros_underlay extension test completed successfully!"
