#!/bin/bash
# Install dependencies for the underlay workspace
# Can be called during Docker build or from inside the container

set -e

# Use workspace layout from environment
UNDERLAY_PATH="${ROS_UNDERLAY_PATH:-/ros_ws/underlay}"

echo "Installing underlay dependencies"

# Check if underlay has any packages
if [ ! -d "${UNDERLAY_PATH}" ] || [ -z "$(find "${UNDERLAY_PATH}" -name 'package.xml' -print -quit)" ]; then
    echo "No packages found in underlay, skipping dependency installation"
    exit 0
fi

cd "${UNDERLAY_PATH}"

# Update rosdep (with caching to speed up repeated builds)
echo "Updating rosdep..."
if [ ! -f /root/.ros/rosdep/sources.cache ] || \
   [ -z "$(find /root/.ros/rosdep/sources.cache -mmin -1440 2>/dev/null)" ]; then
    # Cache is missing or older than 24 hours
    rosdep update
else
    echo "Rosdep cache is recent, skipping update"
fi

# Install dependencies
echo "Installing rosdep dependencies from ${UNDERLAY_PATH}"
rosdep install --from-paths . --ignore-src -y -r --rosdistro "${ROS_DISTRO:-jazzy}"

echo "Dependencies installed successfully"
