#!/bin/bash
set -e

echo "Building ROS underlay from /dependencies..."

# Source ROS setup
source /opt/ros/jazzy/setup.bash

# Check if /dependencies exists and has packages
if [ ! -d "/dependencies" ] || [ "$(find /dependencies -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -eq 0 ]; then
    echo "No packages found in /dependencies, skipping underlay build"
    mkdir -p /opt/ros_underlay
    exit 0
fi

# Update rosdep
echo "Updating rosdep..."
rosdep update || true

# Install dependencies
echo "Installing dependencies with rosdep..."
rosdep install --from-paths /dependencies --ignore-src -y || true

# Build with colcon
echo "Building packages with colcon..."
cd /dependencies
colcon build --install-base /opt/ros_underlay --merge-install

echo "ROS underlay build complete!"
echo "Underlay installed to: /opt/ros_underlay"
echo "Source it with: source /opt/ros_underlay/setup.bash"
