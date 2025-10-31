# Fix ROS Underlay Dependencies and Build Order

## Problem
When using the ros_jazzy extension with a dynamically loaded ROS 2 demo repository (via `renv ros2/demos`), the colcon build fails with CMake errors about missing package configuration files like `example_interfacesConfig.cmake`. The issue is that the underlay build logic is building the wrong underlay workspace.

## Root Cause
The user-side Docker initialization (ros_jazzy_user_snippet.Dockerfile) has logic to build underlay dependencies, but it's checking for packages in the static `/ros_ws/underlay` directory (populated from `consolidated.repos` during Docker build) instead of the dynamically mounted workspace that contains the actual packages being built. When using `renv ros2/demos`, the demos repository gets mounted to the user's workspace directory, but the underlay fix incorrectly builds dependencies for unrelated packages in `/ros_ws/underlay`. This causes:

1. Wrong underlay packages are built (from consolidated.repos instead of current workspace)
2. The actual workspace packages don't have their dependencies installed via rosdep
3. Main workspace colcon build fails because it lacks the correct dependencies

## Solution
Update the bashrc initialization in ros_jazzy_user_snippet.Dockerfile to check for packages in the current workspace directory instead of the static underlay path:

1. Check if the current workspace (`$ROS_WORKSPACE_ROOT`, typically `/ros_ws`) contains packages
2. If it does, run rosdep to install system dependencies for the current workspace
3. Skip the underlay build steps since we're building the main workspace, not an underlay
4. Then proceed with the main colcon build with proper dependencies installed

This ensures that dependencies are installed for the actual packages being built, not unrelated underlay packages.

## Changes
- Modified `ros_jazzy_user_snippet.Dockerfile` to install dependencies for the current workspace instead of building a separate underlay
