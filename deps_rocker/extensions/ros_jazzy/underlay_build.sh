#!/bin/bash
# Build the underlay workspace
# Can be called during Docker build or from inside the container

set -e

# Use workspace layout from environment
UNDERLAY_PATH="${ROS_UNDERLAY_PATH:-/ros_ws/underlay}"
BUILD_BASE="${ROS_BUILD_BASE:-/ros_ws/build}"
INSTALL_BASE="${ROS_INSTALL_BASE:-/ros_ws/install}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "🔨 Building underlay workspace"

# Check if underlay has any packages
if [ ! -d "${UNDERLAY_PATH}" ] || [ -z "$(find "${UNDERLAY_PATH}" -name 'package.xml' -print -quit)" ]; then
    echo "ℹ️  No packages found in underlay, skipping build"
    exit 0
fi

# Source ROS environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "❌ ROS environment not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi

# Build underlay
cd /ros_ws
echo "🏗️  Building packages from ${UNDERLAY_PATH}..."
colcon build \
    --base-paths "${UNDERLAY_PATH}" \
    --build-base "${BUILD_BASE}/underlay" \
    --install-base "${INSTALL_BASE}/underlay" \
    --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "✅ Underlay built successfully"
echo "ℹ️  Source with: source ${INSTALL_BASE}/underlay/setup.bash"
