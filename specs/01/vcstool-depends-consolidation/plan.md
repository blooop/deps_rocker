# Implementation Plan: VCS Tool Depends Consolidation

## Changes Required

### 1. Update vcstool.py
**File:** `deps_rocker/extensions/vcstool/vcstool.py`

- Modify `_get_workspace_layout()` to add `depends_root` path (`/ros_ws/depends`)
- Update `discover_repos()` to:
  - Only discover files named exactly `depends.repos` (not `*.repos` or `depends.repos.yaml`)
  - Set all entries to use the same consolidated path (`depends`)
  - Store original file location for COPY command but use fixed destination for import

### 2. Update vcstool_snippet.Dockerfile
**File:** `deps_rocker/extensions/vcstool/vcstool_snippet.Dockerfile`

- Add environment variable for depends root path
- Update COPY to preserve source paths under repos_root
- Update vcs import and cp commands to always target the consolidated depends_root
- Simplify caching since all imports go to same destination

### 3. Testing Considerations
- Verify multiple depends.repos files all import to /ros_ws/depends/
- Ensure no conflicts when repos overlap between different depends.repos files
- Confirm BuildKit cache still works correctly
- Test that workspace layout is correctly exposed to extensions

## Implementation Steps

1. Update workspace layout in `_get_workspace_layout()`
2. Modify `discover_repos()` to filter and consolidate paths
3. Update Dockerfile snippet to use consolidated import path
4. Run CI tests to verify functionality
5. Update documentation if needed
