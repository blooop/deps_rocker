#!/bin/bash
# Unified script to build underlay workspace
# Part of the ROS Unified Workspace Architecture

set -e

# Use environment variables for unified workspace architecture
UNDERLAY_ROOT="${ROS_UNDERLAY_ROOT:-$HOME/underlay}"
UNDERLAY_SRC="${ROS_UNDERLAY_PATH:-$HOME/underlay/src}"
UNDERLAY_BUILD="${ROS_UNDERLAY_BUILD:-$HOME/underlay/build}"
UNDERLAY_INSTALL="${ROS_UNDERLAY_INSTALL:-$HOME/underlay/install}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "Building underlay workspace from: $UNDERLAY_SRC"

# Check if underlay has any packages
if [ ! -d "$UNDERLAY_SRC" ] || [ -z "$(find "$UNDERLAY_SRC" -name 'package.xml' -print -quit)" ]; then
    echo "No packages found in underlay, skipping build"
    exit 0
fi

# Source ROS environment
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$ROS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    echo "Sourced ROS environment: $ROS_SETUP"
else
    echo "ERROR: ROS environment not found at $ROS_SETUP"
    exit 1
fi

# Create build and install directories
mkdir -p "$UNDERLAY_BUILD" "$UNDERLAY_INSTALL"

# Build underlay packages
echo "Building underlay packages..."
cd "$UNDERLAY_ROOT"
colcon build \
    --base-paths "$UNDERLAY_SRC" \
    --build-base "$UNDERLAY_BUILD" \
    --install-base "$UNDERLAY_INSTALL" \
    --merge-install \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Underlay built successfully!"
echo "To use: source $UNDERLAY_INSTALL/setup.bash"
