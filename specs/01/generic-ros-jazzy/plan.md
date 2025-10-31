# Implementation Plan: Generic ROS Jazzy Extension

## Phase 1: Remove Project-Specific Hacks
1. Remove hardcoded test files from `get_files()` method:
   - `test_package.xml` - project-specific hack
   - `test_setup.py` - project-specific hack
2. Keep only generic configuration files:
   - `colcon-defaults.yaml` - generic colcon configuration
   - `underlay_deps.sh` - script to install underlay dependencies 
   - `underlay_build.sh` - script to build underlay workspace
   - `install_rosdeps.sh` - script to install main workspace dependencies
   - `consolidated.repos` - merged repos from workspace scan

## Phase 2: Update Dockerfile Template
Current approach has project-specific elements. Need to make it fully generic:

1. **Base Setup**: Use standard ROS base image
2. **User Creation**: Generic user setup with proper permissions
3. **Tool Installation**: vcstool, rosdep, build tools with BuildKit cache mounts
4. **Underlay Workspace Creation**: Create `/opt/ros/underlay` with proper ownership
5. **Repository Cloning**: Use vcstool to clone from consolidated.repos into underlay
6. **Dependency Installation**: Use rosdep for both underlay and main workspace
7. **Underlay Build**: Build underlay workspace with colcon
8. **Environment Setup**: Proper bashrc sourcing without $HOME expansion issues

## Phase 3: Fix Environment Variables
- Replace hardcoded `/home/$USERNAME` paths with proper variable expansion
- Ensure ROS_DISTRO is properly set to 'jazzy'
- Fix bashrc sourcing to use absolute paths instead of $HOME

## Phase 4: Script Updates
Update shell scripts to be completely generic:
- `underlay_deps.sh`: Clone repos with vcstool, install rosdep deps for underlay
- `underlay_build.sh`: Build underlay workspace with colcon
- `install_rosdeps.sh`: Install rosdep dependencies for main workspace

## Phase 5: Testing Strategy
1. Test with `pixi r test-extension ros_jazzy` for basic functionality
2. Test with `renv ros2/demos -f -- colcon build` 
3. Test with `renv kinisi-robotics/ros_devcontainer -f -- colcon build`
4. Verify no project-specific dependencies remain

## Key Implementation Notes
- Use BuildKit cache mounts for apt, pip, and vcstool operations
- Ensure proper file permissions and ownership throughout
- Use bind mounts for workspace scanning to avoid bloating image
- Follow the pattern from the provided Dockerfile example for structure