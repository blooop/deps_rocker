#!/bin/bash
set -e

echo "Testing ros_underlay extension..."

# Check if build-underlay command exists
if ! command -v build-underlay &> /dev/null; then
    echo "ERROR: build-underlay command not found"
    exit 1
fi

# Check if /ros_underlay directory was created
if [ ! -d "/ros_underlay" ]; then
    echo "ERROR: /ros_underlay directory not found"
    exit 1
fi

# Test that the build-underlay command runs without error
build-underlay

# Check environment variables are set

if [[ ":$AMENT_PREFIX_PATH:" != *":/ros_underlay:"* ]]; then
    echo "ERROR: AMENT_PREFIX_PATH does not contain /ros_underlay"
    exit 1
fi

echo "ros_underlay extension test completed successfully!"
