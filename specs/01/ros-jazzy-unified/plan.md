# Unified Plan for ROS Jazzy Extension & Tests

## Overview
This plan consolidates all requirements for a generic, dynamic, and robust ros_jazzy extension and its test suite. It replaces redundant specs and ensures all features are covered in a single workflow.

## Steps
1. **Genericize Extension**
   - Remove all project-specific logic/files
   - Ensure get_files() is generic
2. **Dynamic Repo Management**
   - Implement update_repos.sh for runtime repo discovery
   - Remove build-time repo consolidation
3. **Workspace & Mounts**
   - Use $HOME for all workspace paths
   - CWD mounts to ~/project_name
   - Remove workdir extension dependency/conflict
4. **Script Updates**
   - Ensure all scripts (rosdep_underlay.sh, rosdep_overlay.sh, build_underlay.sh, update_repos.sh) work for both build and runtime
5. **Test Suite Overhaul**
   - Use static fixture package for tests
   - Remove symlink/test package hacks
   - Validate with ros2/demos and kinisi-robotics/ros_devcontainer
   - Ensure git worktree is clean after CI
6. **CI & Success Criteria**
   - pixi run ci passes
   - All environment variables and mounts correct
   - No leftover test artifacts
   - All features validated by tests

## Deliverables
- Unified spec.md and plan.md
- Remove/mark redundant specs for deletion
- Commit only the new consolidated spec folder
