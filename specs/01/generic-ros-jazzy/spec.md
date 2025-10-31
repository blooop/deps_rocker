# Generic ROS Jazzy Extension

## Problem
The current ros_jazzy extension contains project-specific hardcoded files (test_package.xml, setup_py_template) that make it non-generic. It should work with any ROS repository without requiring project-specific hacks.

## Solution
Create a completely generic ROS Jazzy extension that:

1. **Scans for *.repos files** in the workspace and consolidates them
2. **Clones dependencies** using vcstool into an underlay workspace (`/opt/ros/underlay`)
3. **Installs rosdep dependencies** for the underlay workspace
4. **Builds the underlay workspace** with colcon
5. **Installs rosdep dependencies** for the main workspace packages
6. **Sets up proper environment sourcing** for ROS, underlay, and main workspace

## Key Requirements
- Zero project-specific hardcoded files
- Works with arbitrary ROS repositories (ros2/demos, kinisi-robotics/ros_devcontainer, etc.)
- Proper environment variable handling (avoid $HOME expansion issues)
- Uses vcstool for repository cloning
- Proper BuildKit cache mounting for performance
- Generic rosdep dependency resolution

## Success Criteria
- `renv ros2/demos -f -- colcon build` works
- `renv kinisi-robotics/ros_devcontainer -f -- colcon build` works
- `pixi r test-extension ros_jazzy` passes
- No hardcoded project-specific files in get_files() method