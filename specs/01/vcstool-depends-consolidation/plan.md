# Implementation Plan: VCS Tool Depends Consolidation

## Changes Required

### 1. Simplify vcstool.py
**File:** `deps_rocker/extensions/vcstool/vcstool.py`

- Remove empy-based workspace layout methods
- Remove `_get_workspace_layout()` and `_ensure_empy_args()` methods
- Implement simple `get_files()` method that:
  - Scans for all `depends.repos` files
  - Merges all repositories into single `consolidated.repos` manifest
  - Returns the consolidated manifest as a YAML file

### 2. Simplify vcstool_snippet.Dockerfile
**File:** `deps_rocker/extensions/vcstool/vcstool_snippet.Dockerfile`

- Define only essential workspace env vars:
  - `ROS_WORKSPACE_ROOT=/ros_ws`
  - `ROS_UNDERLAY_PATH=/ros_ws/underlay`
  - `ROS_BUILD_BASE`, `ROS_INSTALL_BASE`, `ROS_LOG_BASE`
- Create only necessary directories (no repos, src, or depends dirs)
- Import consolidated.repos directly to `/ros_ws/underlay`
- Use BuildKit cache for vcs clones

### 3. Simplify ros_jazzy_snippet.Dockerfile
**File:** `deps_rocker/extensions/ros_jazzy/ros_jazzy_snippet.Dockerfile`

- Remove duplicate workspace env vars (now defined by vcstool)
- Remove duplicate directory creation
- Only define colcon-specific variables

## Implementation Steps

1. Rewrite vcstool.py to merge all depends.repos into single manifest
2. Update vcstool Dockerfile to import to underlay with minimal env vars
3. Update ros_jazzy Dockerfile to remove duplicates
4. Commit with updated spec and plan
