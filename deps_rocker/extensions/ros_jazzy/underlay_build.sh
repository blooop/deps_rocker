#!/bin/bash
# Generic script to build the underlay workspace
# Works with any ROS repository structure

set -e

# Use environment variables or defaults for unified workspace architecture
UNDERLAY_SRC="${ROS_UNDERLAY_PATH:-$HOME/underlay/src}"
UNDERLAY_BUILD="${ROS_UNDERLAY_BUILD:-$HOME/underlay/build}"
UNDERLAY_INSTALL="${ROS_UNDERLAY_INSTALL:-$HOME/underlay/install}"
UNDERLAY_ROOT="${ROS_UNDERLAY_ROOT:-$HOME/underlay}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "Building underlay workspace from: $UNDERLAY_SRC"

# Check if underlay has any packages
if [ ! -d "$UNDERLAY_SRC" ] || [ -z "$(find "$UNDERLAY_SRC" -name 'package.xml' -print -quit)" ]; then
    echo "No packages found in underlay, skipping build"
    exit 0
fi

# Source ROS environment
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"

# Check if we have a custom ROS_SETUP_SCRIPT for testing
if [ -n "$ROS_SETUP_SCRIPT" ] && [ -f "$ROS_SETUP_SCRIPT" ]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP_SCRIPT"
    echo "Sourced custom ROS environment: $ROS_SETUP_SCRIPT"
elif [ -f "$ROS_SETUP" ]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    echo "Sourced ROS environment: $ROS_SETUP"
else
    echo "ERROR: ROS environment not found at $ROS_SETUP"
    exit 1
fi

# Clean and create build and install directories to avoid conflicts
rm -rf "$UNDERLAY_BUILD" "$UNDERLAY_INSTALL"
mkdir -p "$UNDERLAY_BUILD" "$UNDERLAY_INSTALL"

# Set world-accessible permissions to ensure containers can access these directories
chmod 777 "$UNDERLAY_BUILD" "$UNDERLAY_INSTALL"

# Build underlay packages
echo "Building underlay packages..."
# Use parent directory of UNDERLAY_SRC if UNDERLAY_ROOT doesn't exist
if [ -d "$UNDERLAY_ROOT" ]; then
    cd "$UNDERLAY_ROOT"
else
    cd "$(dirname "$UNDERLAY_SRC")"
fi
colcon build \
    --base-paths "$UNDERLAY_SRC" \
    --build-base "$UNDERLAY_BUILD" \
    --install-base "$UNDERLAY_INSTALL" \
    --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Underlay built successfully!"
echo "To use: source $UNDERLAY_INSTALL/setup.bash"
