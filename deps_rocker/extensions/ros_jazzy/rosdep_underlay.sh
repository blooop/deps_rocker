#!/bin/bash
# Unified script to install rosdep dependencies for underlay workspace
# Part of the ROS Unified Workspace Architecture

set -e

# Use environment variables for unified workspace architecture
UNDERLAY_PATH="${ROS_UNDERLAY_PATH:-$HOME/underlay/src}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "Installing rosdep dependencies for underlay workspace: $UNDERLAY_PATH"

# Check if underlay has any packages
if [ ! -d "$UNDERLAY_PATH" ] || [ -z "$(find "$UNDERLAY_PATH" -name 'package.xml' -print -quit)" ]; then
    echo "No packages found in underlay, skipping rosdep installation"
    exit 0
fi

# Change to underlay directory
cd "$UNDERLAY_PATH"

# Update rosdep database
echo "Updating rosdep database..."
rosdep update || echo "Warning: rosdep update failed, continuing..."

# Install dependencies
echo "Installing rosdep dependencies for underlay..."
export DEBIAN_FRONTEND=noninteractive
rosdep install --from-paths . --ignore-src -y -r --rosdistro "$ROS_DISTRO"

echo "Underlay rosdep dependencies installed successfully"