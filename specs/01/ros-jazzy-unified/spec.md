# Unified ROS Jazzy Extension Specification

## Purpose
Create a generic, robust ROS 2 Jazzy extension and test suite that:
- Works with any ROS repository (no project-specific hacks)
- Supports dynamic repository management (no stale .repos)
- Provides correct underlay/overlay workspace structure
- Handles $HOME and mount paths correctly
- Ensures clean CI/test runs (no leftover artifacts)
- Removes symlink/test package hacks

## Key Features
1. **Generic Repository Handling**
   - Scans for *.repos files in workspace
   - Uses vcstool for underlay cloning
   - No hardcoded files or paths
2. **Dynamic Repo Management**
   - Processes *.repos at runtime (not build time)
   - update_repos.sh script for live updates
3. **Workspace Architecture**
   - Underlay: dependencies from repos
   - Overlay: user packages (cwd auto-mounted)
   - Both use $HOME for all paths
   - No reliance on /ros_ws or static paths
4. **Environment & Mounts**
   - All mounts inside $HOME
   - CWD extension mounts to ~/project_name
   - No workdir conflicts with other extensions
5. **Test & CI Cleanliness**
   - Tests use static fixture package (no auto-gen)
   - No symlink hacks
   - After CI, git worktree is clean
6. **Script Architecture**
   - rosdep_underlay.sh, rosdep_overlay.sh, build_underlay.sh, update_repos.sh
   - All scripts work for both build-time and runtime
7. **Success Criteria**
   - renv ros2/demos -f -- colcon build works
   - renv kinisi-robotics/ros_devcontainer -f -- colcon build works
   - pixi run ci passes
   - Investigate current `pixi run ci` failure and resolve underlying issues
   - No hardcoded files in get_files()
   - All environment variables and mounts correct
   - No leftover test artifacts
   - All features validated by comprehensive test suite
