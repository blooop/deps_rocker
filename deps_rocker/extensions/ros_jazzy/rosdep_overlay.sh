#!/bin/bash
# Unified script to install rosdep dependencies for overlay workspace
# Part of the ROS Unified Workspace Architecture

set -e

# Use environment variables for unified workspace architecture
OVERLAY_PATH="${ROS_WORKSPACE_ROOT:-$HOME/overlay}/src"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "Installing rosdep dependencies for overlay workspace: $OVERLAY_PATH"

# Check if overlay has any packages
if [ ! -d "$OVERLAY_PATH" ] || [ -z "$(find "$OVERLAY_PATH" -name 'package.xml' -print -quit)" ]; then
    echo "No packages found in overlay, skipping rosdep installation"
    exit 0
fi

# Change to overlay directory
cd "$OVERLAY_PATH"

# Update rosdep database
echo "Updating rosdep database..."
rosdep update || echo "Warning: rosdep update failed, continuing..."

# Install dependencies
echo "Installing rosdep dependencies for overlay..."
export DEBIAN_FRONTEND=noninteractive
rosdep install --from-paths . --ignore-src -y -r --rosdistro "$ROS_DISTRO"

echo "Overlay rosdep dependencies installed successfully"