#!/bin/bash
# Build the underlay workspace
# Runs in user context to avoid ownership issues

set -e

# Use workspace layout from environment
UNDERLAY_PATH="${ROS_UNDERLAY_PATH:-/ros_ws/underlay/src}"
UNDERLAY_BUILD="${ROS_UNDERLAY_BUILD:-/ros_ws/underlay/build}"
UNDERLAY_INSTALL="${ROS_UNDERLAY_INSTALL:-/ros_ws/underlay/install}"
WORKSPACE_ROOT="${ROS_WORKSPACE_ROOT:-/ros_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP_SCRIPT="${ROS_SETUP_SCRIPT:-/opt/ros/${ROS_DISTRO}/setup.bash}"

echo "Building underlay workspace"

# Check if underlay has any packages
if [ ! -d "${UNDERLAY_PATH}" ] || [ -z "$(find "${UNDERLAY_PATH}" -name 'package.xml' -print -quit)" ]; then
    echo "No packages found in underlay, skipping build"
    exit 0
fi

# Source ROS environment
if [ -f "${ROS_SETUP_SCRIPT}" ]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP_SCRIPT}"
else
    echo "ROS environment not found at ${ROS_SETUP_SCRIPT}"
    exit 1
fi

# Clean out any stale artifacts from previous builds and recreate with correct permissions
if [ -d "${UNDERLAY_BUILD}" ]; then
    rm -rf "${UNDERLAY_BUILD:?}/"*
fi
if [ -d "${UNDERLAY_INSTALL}" ]; then
    rm -rf "${UNDERLAY_INSTALL:?}/"*
fi

# Create directories with world-accessible permissions in one step
install -d -m 777 "${UNDERLAY_BUILD}" "${UNDERLAY_INSTALL}"

# Build underlay
cd "${WORKSPACE_ROOT}"
echo "Building packages from ${UNDERLAY_PATH}..."
colcon build \
    --base-paths "${UNDERLAY_PATH}" \
    --build-base "${UNDERLAY_BUILD}" \
    --install-base "${UNDERLAY_INSTALL}" \
    --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Underlay built successfully"
echo "Source with: source ${UNDERLAY_INSTALL}/setup.bash"
