# Implementation Plan: Fix ROS Underlay Dependencies and Build Order

## Step 1: Analyze the Issue
- [x] Review ros_jazzy extension code
- [x] Identify the problem in ros_jazzy_user_snippet.Dockerfile
- [x] Understand the execution flow of underlay vs main workspace builds

## Step 2: Implement the Fix
- [x] Update ros_jazzy_user_snippet.Dockerfile to call underlay_deps.sh and underlay_build.sh before main colcon build
- [x] Add checks to only run these scripts if the underlay contains packages
- [x] Use `|| true` to gracefully handle cases where there's nothing to build

## Step 3: Testing
- [ ] Run `pixi run ci` to verify formatting, linting, and tests pass
- [ ] If CI passes, commit the changes

## Technical Details

### What Changed
In `ros_jazzy_user_snippet.Dockerfile:20`, the initialization script now includes:

```bash
if [ -d "${ROS_UNDERLAY_PATH:-/ros_ws/underlay}" ] && [ -n "$(find "${ROS_UNDERLAY_PATH:-/ros_ws/underlay}" -name "package.xml" -print -quit)" ]; then
    echo "Building underlay workspace first..."
    underlay_deps.sh || true
    underlay_build.sh || true
fi
```

This ensures that:
1. Underlay system dependencies are installed first
2. Underlay packages are compiled before the main workspace build
3. CMake configuration files are available for the main build to find

### Why This Works
- The underlay_deps.sh script uses rosdep to install system-level dependencies needed by the underlay packages
- The underlay_build.sh script compiles the underlay packages, generating the required CMake config files (e.g., example_interfacesConfig.cmake)
- The main colcon build can now find these config files because they exist and the underlay is sourced
