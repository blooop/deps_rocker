#!/bin/bash
# Unified script to install rosdep dependencies for overlay workspace
# Part of the ROS Unified Workspace Architecture

set -e

# Use environment variables for unified workspace architecture
OVERLAY_PATH="${ROS_WORKSPACE_ROOT:-$HOME/overlay}/src"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "Installing rosdep dependencies for overlay workspace..."

# Check multiple possible locations for packages:
# 1. Current working directory (for renv workflow)
# 2. Standard overlay location
# 3. If current directory is a ROS workspace, use it

TARGET_PATH=""

# First check if current directory has ROS packages
if [ -n "$(find . -maxdepth 3 -name 'package.xml' -print -quit 2>/dev/null)" ]; then
    TARGET_PATH="$(pwd)"
    echo "Found packages in current directory: $TARGET_PATH"
elif [ -d "$OVERLAY_PATH" ] && [ -n "$(find "$OVERLAY_PATH" -name 'package.xml' -print -quit 2>/dev/null)" ]; then
    TARGET_PATH="$OVERLAY_PATH"
    echo "Found packages in overlay: $TARGET_PATH"
else
    echo "No packages found in current directory or overlay, skipping rosdep installation"
    exit 0
fi

# Change to target directory
cd "$TARGET_PATH"

# Update rosdep database
echo "Updating rosdep database..."
rosdep update || echo "Warning: rosdep update failed, continuing..."

# Install dependencies
echo "Installing rosdep dependencies for overlay..."
export DEBIAN_FRONTEND=noninteractive
rosdep install --from-paths . --ignore-src -y -r --verbose --rosdistro "$ROS_DISTRO"

echo "Overlay rosdep dependencies installed successfully"