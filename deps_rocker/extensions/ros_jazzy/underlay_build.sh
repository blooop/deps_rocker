#!/bin/bash
# Build the underlay workspace
# Can be called during Docker build or from inside the container

set -e

# Use workspace layout from environment
UNDERLAY_PATH="${ROS_UNDERLAY_PATH:-/ros_ws/underlay}"
UNDERLAY_BUILD="${ROS_UNDERLAY_BUILD:-/ros_ws/underlay_build}"
UNDERLAY_INSTALL="${ROS_UNDERLAY_INSTALL:-/ros_ws/underlay_install}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "Building underlay workspace"

# Check if underlay has any packages
if [ ! -d "${UNDERLAY_PATH}" ] || [ -z "$(find "${UNDERLAY_PATH}" -name 'package.xml' -print -quit)" ]; then
    echo "No packages found in underlay, skipping build"
    exit 0
fi

# Source ROS environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "ROS environment not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi

# Create build and install directories if they don't exist
mkdir -p "${UNDERLAY_BUILD}" "${UNDERLAY_INSTALL}"

# Build underlay
cd /ros_ws
echo "Building packages from ${UNDERLAY_PATH}..."
colcon build \
    --base-paths "${UNDERLAY_PATH}" \
    --build-base "${UNDERLAY_BUILD}" \
    --install-base "${UNDERLAY_INSTALL}" \
    --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Ensure built artifacts are accessible by all users
# This is critical for non-root users to source and use the underlay
chmod -R a+rX "${UNDERLAY_BUILD}" "${UNDERLAY_INSTALL}"

echo "Underlay built successfully"
echo "Source with: source ${UNDERLAY_INSTALL}/setup.bash"
