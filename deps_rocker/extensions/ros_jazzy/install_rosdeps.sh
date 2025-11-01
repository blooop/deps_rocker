#!/bin/bash
# Generic script to install rosdeps for any ROS workspace
# Searches current directory for ROS packages and installs their dependencies

set -e

WORKSPACE_PATH="${WORKSPACE_PATH:-$(pwd)}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "Installing rosdeps for workspace: $WORKSPACE_PATH"

# Check if workspace has ROS packages
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "Workspace directory does not exist: $WORKSPACE_PATH"
    exit 0
fi

# Find ROS packages
PACKAGE_PATHS=$(find "$WORKSPACE_PATH" -name "package.xml" -exec dirname {} \; 2>/dev/null || true)

if [ -z "$PACKAGE_PATHS" ]; then
    echo "No ROS packages found in workspace, skipping rosdeps installation"
    exit 0
fi

echo "Found ROS packages:"
echo "$PACKAGE_PATHS" | while read -r dir; do
    echo "  - $dir"
done

# Update rosdep database
echo "Updating rosdep database..."
rosdep update || echo "Warning: rosdep update failed, continuing..."

# Install dependencies for all packages at once (more efficient)
echo "Installing rosdep dependencies..."
export DEBIAN_FRONTEND=noninteractive
echo "$PACKAGE_PATHS" | xargs rosdep install --from-paths --ignore-src -r -y --rosdistro "$ROS_DISTRO" || {
    echo "Warning: Some rosdep dependencies failed to install, continuing..."
}

echo "Rosdeps installation completed"
