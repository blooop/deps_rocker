# VCS Tool Depends Consolidation

## Objective
Consolidate all discovered depends.repos files to import into `/ros_ws/underlay` directory. Unify workspace environment variables with ros_jazzy extension using hard-coded paths (no empy).

## Current Behavior
When vcstool discovers multiple `depends.repos` files (e.g., `foo/depends.repos`, `bar/depends.repos`), each imports to its own path:
- `foo/depends.repos` → imports to `/ros_ws/src/foo/`
- `bar/depends.repos` → imports to `/ros_ws/src/bar/`
- Uses empy variables for workspace paths

## New Behavior
All `depends.repos` files should import to the underlay:
- All discovered `depends.repos` → merge and import to `/ros_ws/underlay`
- Use hard-coded paths (no empy variables)
- Only define essential workspace env vars: `ROS_WORKSPACE_ROOT`, `ROS_UNDERLAY_PATH`

## Rationale
- Simplest workspace layout: dependencies go directly into underlay
- Avoids path conflicts when multiple manifests define overlapping repos
- Consistent with ros_jazzy approach (no empy, hard-coded paths)
- No unnecessary intermediate directories

## Implementation
Update vcstool extension to:
1. Scan for all `depends.repos` files and merge into single manifest
2. Import merged manifest to `/ros_ws/underlay`
3. Remove empy-based workspace layout computation
4. Define only essential workspace env vars
