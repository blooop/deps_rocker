#!/bin/bash
# Install rosdeps for ROS workspace packages
# This script finds all ROS packages in the current workspace and installs their dependencies

set -e

# Default workspace path
WORKSPACE_PATH="${WORKSPACE_PATH:-$HOME/demos}"
ROSDEPS_MARKER="$HOME/.rosdeps_installed"

# Function to print with timestamp
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

# Check if workspace exists
if [ ! -d "$WORKSPACE_PATH" ]; then
    log "Workspace directory $WORKSPACE_PATH does not exist, skipping rosdeps installation"
    exit 0
fi

# Check if already installed
if [ -f "$ROSDEPS_MARKER" ]; then
    log "Rosdeps already installed (marker file exists: $ROSDEPS_MARKER)"
    exit 0
fi

log "Installing rosdeps for workspace packages in $WORKSPACE_PATH..."

# Update rosdep database
log "Updating rosdep database..."
rosdep update 2>/dev/null || rosdep update 2>/dev/null || {
    log "Warning: Failed to update rosdep database, continuing..."
}

# Find all ROS packages in workspace
log "Searching for ROS packages..."
WORKSPACE_DIRS=$(find "$WORKSPACE_PATH" -maxdepth 3 -name "package.xml" -exec dirname {} \; | head -30)

if [ -z "$WORKSPACE_DIRS" ]; then
    log "No ROS packages found in workspace $WORKSPACE_PATH"
    # Still create marker to avoid repeated checks
    touch "$ROSDEPS_MARKER"
    exit 0
fi

log "Found workspace packages:"
echo "$WORKSPACE_DIRS" | while read -r dir; do
    log "  - $dir"
done

# Install dependencies for each package
log "Installing rosdep dependencies..."
echo "$WORKSPACE_DIRS" | while read -r pkg_dir; do
    if [ -n "$pkg_dir" ] && [ -d "$pkg_dir" ]; then
        log "Installing dependencies for $pkg_dir..."
        rosdep install --from-paths "$pkg_dir" --ignore-src -r -y --rosdistro "${ROS_DISTRO:-jazzy}" \
            --skip-keys="example_interfaces" 2>/dev/null || {
            log "Warning: Some dependencies failed to install for $pkg_dir, continuing..."
        }
    fi
done

# Create marker file to indicate completion
touch "$ROSDEPS_MARKER"
log "Rosdeps installation completed successfully"