# ROS Jazzy $HOME Variable Fix Implementation Plan

## Current Issue
The `ros_jazzy_user_snippet.Dockerfile` fails with permission errors because the mkdir command uses environment variables that still reference the old `/ros_ws` paths instead of the updated `$HOME/ros_ws` paths.

## Root Cause Analysis
1. Main snippet sets ENV variables to `/ros_ws` (absolute paths)
2. User snippet redefines ENV variables to `$HOME/ros_ws`
3. User snippet immediately uses the ENV variables in mkdir command
4. Docker may not have processed the ENV changes in the same build context
5. mkdir command uses old `/ros_ws` paths, causing permission errors

## Simple Solution
Replace the mkdir command to use `$HOME` directly instead of relying on environment variables:

### Before (problematic):
```dockerfile
ENV ROS_WORKSPACE_ROOT=$HOME/ros_ws
ENV ROS_UNDERLAY_PATH=$HOME/ros_ws/underlay
# ... other ENV statements ...
RUN mkdir -p "$ROS_UNDERLAY_PATH" "$ROS_UNDERLAY_BUILD" "$ROS_UNDERLAY_INSTALL" \
  "$ROS_BUILD_BASE" "$ROS_INSTALL_BASE" "$ROS_LOG_BASE"
```

### After (working):
```dockerfile
ENV ROS_WORKSPACE_ROOT=$HOME/ros_ws
ENV ROS_UNDERLAY_PATH=$HOME/ros_ws/underlay
# ... other ENV statements ...
RUN mkdir -p "$HOME/ros_ws/underlay" "$HOME/ros_ws/underlay_build" "$HOME/ros_ws/underlay_install" \
  "$HOME/ros_ws/build" "$HOME/ros_ws/install" "$HOME/ros_ws/log"
```

## Files to Modify
1. `deps_rocker/extensions/ros_jazzy/ros_jazzy_user_snippet.Dockerfile` - Replace environment variable references with direct $HOME paths in mkdir command

## Validation
- Build should complete without permission errors
- ROS workspace should be created in user home directory
- Environment variables should still work correctly for other commands