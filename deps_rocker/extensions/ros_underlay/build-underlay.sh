#!/bin/bash
set -e

echo "Building ROS underlay from vcstool repositories..."

# Source ROS setup

source /opt/ros/jazzy/setup.bash

# Find all *.repos and depends.repos.yaml files and extract their parent directories
repos_paths=()
if [ -d "/dependencies" ]; then
    while IFS= read -r -d '' repos_file; do
        repos_dir=$(dirname "$repos_file")
        if [ -d "$repos_dir" ] && [ "$(find "$repos_dir" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -gt 0 ]; then
            repos_paths+=("$repos_dir")
        fi
    done < <(find /dependencies \( -name "*.repos" -o -name "depends.repos.yaml" \) -type f -print0)
fi

# Check if we found any packages
if [ ${#repos_paths[@]} -eq 0 ]; then
    echo "No packages found from *.repos or depends.repos.yaml files, skipping underlay build"
    mkdir -p /ros_underlay
    exit 0
fi

echo "Found ${#repos_paths[@]} repository path(s) to build"

# Update rosdep
echo "Updating rosdep..."
rosdep update || true

# Install dependencies
echo "Installing dependencies with rosdep..."
for path in "${repos_paths[@]}"; do
    echo "  Installing dependencies from: $path"
    rosdep install --from-paths "$path" --ignore-src -y || true
done

# Build with colcon
echo "Building packages with colcon..."
mkdir -p /ros_underlay

for path in "${repos_paths[@]}"; do
    echo "  Building packages from: $path"
    cd "$path"
    colcon build --install-base /ros_underlay --merge-install
done

echo "ROS underlay build complete!"
echo "Underlay installed to: /ros_underlay"
echo "Source it with: source /ros_underlay/setup.bash"
