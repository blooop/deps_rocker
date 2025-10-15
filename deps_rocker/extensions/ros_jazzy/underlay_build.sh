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

CURRENT_UID="$(id -u)"
CURRENT_GID="$(id -g)"

ensure_owner_for_paths() {
    local paths=("$@")
    if [ "${CURRENT_UID}" -eq 0 ]; then
        chown -R "${CURRENT_UID}:${CURRENT_GID}" "${paths[@]}"
    else
        if command -v sudo >/dev/null 2>&1; then
            sudo chown -R "${CURRENT_UID}:${CURRENT_GID}" "${paths[@]}"
        else
            echo "ERROR: sudo is required to adjust underlay ownership when running as a non-root user."
            exit 1
        fi
    fi
}

ensure_world_accessible() {
    local paths=("$@")
    if [ "${CURRENT_UID}" -eq 0 ]; then
        chmod -R a+rwX "${paths[@]}"
    else
        sudo chmod -R a+rwX "${paths[@]}"
    fi
}

# Ensure the active user owns the build/install directories before building so CMake can modify timestamps
ensure_owner_for_paths "${UNDERLAY_BUILD}" "${UNDERLAY_INSTALL}"

# Clean out any stale artifacts left by previous builds (especially ones created by a different user)
if [ -n "$(ls -A "${UNDERLAY_BUILD}")" ]; then
    rm -rf "${UNDERLAY_BUILD:?}/"*
fi
if [ -n "$(ls -A "${UNDERLAY_INSTALL}")" ]; then
    rm -rf "${UNDERLAY_INSTALL:?}/"*
fi

# Build underlay
cd /ros_ws
echo "Building packages from ${UNDERLAY_PATH}..."
colcon build \
    --base-paths "${UNDERLAY_PATH}" \
    --build-base "${UNDERLAY_BUILD}" \
    --install-base "${UNDERLAY_INSTALL}" \
    --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Ensure built artifacts are accessible/writeable by all users. CMake expects to be able to update timestamps,
# so we need both ownership (handled above) and world-readable/executable bits for follow-up consumers.
ensure_world_accessible "${UNDERLAY_BUILD}" "${UNDERLAY_INSTALL}"

echo "Underlay built successfully"
echo "Source with: source ${UNDERLAY_INSTALL}/setup.bash"
