# VCS Tool Depends Consolidation

## Objective
Consolidate all discovered depends.repos files to import into a single `/ros_ws/depends/` directory instead of mirroring the source directory structure. Unify workspace environment variables with ros_jazzy extension using hard-coded paths (no empy).

## Current Behavior
When vcstool discovers multiple `depends.repos` files (e.g., `foo/depends.repos`, `bar/depends.repos`), each imports to its own path:
- `foo/depends.repos` → imports to `/ros_ws/src/foo/`
- `bar/depends.repos` → imports to `/ros_ws/src/bar/`
- Uses empy variables for workspace paths

## New Behavior
All `depends.repos` files should import to a single consolidated location:
- All discovered `depends.repos` → import to `/ros_ws/depends/`
- Use hard-coded paths matching ros_jazzy (no empy variables)
- Add ROS_DEPENDS_ROOT environment variable

## Rationale
- Simpler workspace layout with dependencies in a dedicated directory
- Avoids path conflicts when multiple manifests define overlapping repos
- Clear separation between project source code and external dependencies
- Consistent with ros_jazzy approach (no empy, hard-coded paths)

## Implementation
Update vcstool extension to:
1. Filter discovered repos to only process files named `depends.repos`
2. Import all such files to `/ros_ws/depends/` regardless of their source location
3. Remove empy-based workspace layout computation
4. Hard-code workspace paths in Dockerfile like ros_jazzy does
