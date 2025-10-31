#!/bin/bash
# Build the underlay workspace
# Can be called during Docker build or from inside the container

set -e

# Use workspace layout from environment
UNDERLAY_PATH="${ROS_UNDERLAY_PATH:-/home/user/ros_ws/underlay}"
UNDERLAY_BUILD="${ROS_UNDERLAY_BUILD:-/home/user/ros_ws/underlay_build}"
UNDERLAY_INSTALL="${ROS_UNDERLAY_INSTALL:-/home/user/ros_ws/underlay_install}"
WORKSPACE_ROOT="${ROS_WORKSPACE_ROOT:-/home/user/ros_ws}"
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

# Create build and install directories if they don't exist
mkdir -p "${UNDERLAY_BUILD}" "${UNDERLAY_INSTALL}"

CURRENT_UID="$(id -u)"
CURRENT_GID="$(id -g)"

ensure_owner_for_paths() {
    local path current_owner target_owner
    target_owner="${CURRENT_UID}:${CURRENT_GID}"
    for path in "$@"; do
        [ ! -e "${path}" ] && continue
        current_owner="$(stat -c '%u:%g' "${path}")"
        if [ "${current_owner}" = "${target_owner}" ]; then
            continue
        fi

        if [ "${CURRENT_UID}" -eq 0 ]; then
            chown -R "${target_owner}" "${path}"
        elif command -v sudo >/dev/null 2>&1; then
            sudo chown -R "${target_owner}" "${path}"
        else
            echo "ERROR: Unable to adjust ownership for ${path}; run as root or install sudo." >&2
            exit 1
        fi
    done
}

ensure_world_accessible() {
    local path
    for path in "$@"; do
        [ ! -e "${path}" ] && continue
        if chmod -R a+rwX "${path}" 2>/dev/null; then
            continue
        fi

        if [ "${CURRENT_UID}" -eq 0 ]; then
            chmod -R a+rwX "${path}"
        elif command -v sudo >/dev/null 2>&1; then
            sudo chmod -R a+rwX "${path}"
        else
            echo "WARNING: Unable to adjust permissions for ${path}; some users may lack access." >&2
        fi
    done
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
cd "${WORKSPACE_ROOT}"
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
