# Fix ROS Underlay Dependencies and Build Order

## Problem
When using the ros_jazzy extension with a dynamically loaded ROS 2 demo repository (via `auto=/path/to/demos`), the colcon build fails with CMake errors about missing package configuration files like `example_interfacesConfig.cmake`. The underlay dependencies are not being properly installed or the underlay is not being built before the main workspace build attempts to find those packages.

## Root Cause
The user-side Docker initialization (ros_jazzy_user_snippet.Dockerfile) only sources the underlay if it already exists from the build stage, but when the underlay is provided at runtime (via auto parameter), the `underlay_deps.sh` and `underlay_build.sh` scripts are never executed in the user container environment. This causes:

1. System dependencies for underlay packages are never installed via rosdep
2. Underlay packages are never compiled, so CMake config files don't exist
3. Main workspace colcon build fails trying to find transitive dependencies from the underlay

## Solution
Update the bashrc initialization in ros_jazzy_user_snippet.Dockerfile to explicitly call the underlay helper scripts before the main colcon build:

1. Check if the underlay directory contains packages (package.xml files)
2. If it does, run `underlay_deps.sh` to install system dependencies via rosdep
3. Run `underlay_build.sh` to compile the underlay packages
4. Source the underlay install setup.bash
5. Then proceed with the main colcon build

This ensures proper dependency resolution and package availability before the main build starts.

## Changes
- Modified `ros_jazzy_user_snippet.Dockerfile:20` to add pre-build checks and underlay building
