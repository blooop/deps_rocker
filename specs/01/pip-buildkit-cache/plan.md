# Plan: Pip BuildKit Cache Mounts

## Background
Currently, all `pip install` commands in Dockerfiles download packages fresh on every build. BuildKit cache mounts allow pip's download cache to persist across builds, significantly speeding up rebuild times.

## Files to Modify

### 1. deps_rocker/templates/vcstool_snippet.Dockerfile
- Line 7: `RUN pip install vcstool --break-system-packages`
- Change to: `RUN --mount=type=cache,target=/root/.cache/pip pip install vcstool --break-system-packages`

### 2. deps_rocker/extensions/vcstool/vcstool_snippet.Dockerfile
- Line 1: `RUN pip install vcstool --break-system-packages`
- Change to: `RUN --mount=type=cache,target=/root/.cache/pip pip install vcstool --break-system-packages`

### 3. deps_rocker/extensions/ros_jazzy/ros_jazzy_snippet.Dockerfile
- Line 45: `RUN pip install colcon-common-extensions colcon-defaults colcon-spawn-shell colcon-runner colcon-clean rosdep colcon-top-level-workspace --break-system-packages`
- Change to: `RUN --mount=type=cache,target=/root/.cache/pip pip install colcon-common-extensions colcon-defaults colcon-spawn-shell colcon-runner colcon-clean rosdep colcon-top-level-workspace --break-system-packages`

### 4. deps_rocker/extensions/isaac_sim/isaacsim_snippet.Dockerfile
- Line 5: `RUN pip install isaacsim[all,extscache]==4.5.0`
- Change to: `RUN --mount=type=cache,target=/root/.cache/pip pip install isaacsim[all,extscache]==4.5.0`

## Implementation Steps
1. Update all 4 Dockerfiles with the cache mount syntax
2. Run `pixi run ci` to verify changes
3. Fix any issues that arise
4. Commit when all tests pass

## Testing
- Verify that Docker builds still succeed
- Verify that subsequent builds are faster due to cache reuse
- Run full CI suite to ensure no regressions

## Notes
- The cache mount is placed at `/root/.cache/pip`, which is pip's default cache location
- This change is backwards compatible - builds will work with or without BuildKit enabled
- With BuildKit enabled, builds will be significantly faster on subsequent runs
