#!/bin/bash
# Generic script to install dependencies for the underlay workspace
# Works with any ROS repository that uses *.repos dependencies

set -e

# Use environment variable for unified workspace architecture
UNDERLAY_PATH="${ROS_UNDERLAY_PATH:-/home/@(name)/underlay/src}"

echo "Installing underlay dependencies from: $UNDERLAY_PATH"

# Check if underlay has any packages
if [ ! -d "$UNDERLAY_PATH" ] || [ -z "$(find "$UNDERLAY_PATH" -name 'package.xml' -print -quit)" ]; then
    echo "No packages found in underlay, skipping dependency installation"
    exit 0
fi

# Change to underlay directory
cd "$UNDERLAY_PATH"

# Update rosdep database
echo "Updating rosdep database..."
rosdep update || echo "Warning: rosdep update failed, continuing..."

# Install dependencies with BuildKit-friendly apt cache handling
echo "Installing rosdep dependencies..."
export DEBIAN_FRONTEND=noninteractive
rosdep install --from-paths . --ignore-src -y -r --rosdistro "${ROS_DISTRO:-jazzy}"

echo "Underlay dependencies installed successfully"
