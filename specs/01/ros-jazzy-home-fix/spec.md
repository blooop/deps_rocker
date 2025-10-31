# ROS Jazzy $HOME Variable Fix

## Problem
The `ros_jazzy_user_snippet.Dockerfile` uses environment variables (`$ROS_UNDERLAY_PATH`, etc.) in the mkdir command that reference the old `/ros_ws` paths from the main snippet, even though the user snippet redefines them to use `$HOME/ros_ws`. This causes permission errors when trying to create directories like `/ros_ws` instead of the intended `~/ros_ws`.

## Root Cause
The main snippet sets environment variables to absolute paths like `/ros_ws`. The user snippet then redefines these to use `$HOME/ros_ws`, but immediately uses the environment variables in a mkdir command. Docker may not have fully processed the ENV changes within the same build context, causing it to use the old values.

## Solution
Use `$HOME` directly in the mkdir command instead of relying on the environment variables that were just redefined:

```dockerfile
# Instead of:
RUN mkdir -p "$ROS_UNDERLAY_PATH" "$ROS_UNDERLAY_BUILD" "$ROS_UNDERLAY_INSTALL" \
  "$ROS_BUILD_BASE" "$ROS_INSTALL_BASE" "$ROS_LOG_BASE"

# Use:
RUN mkdir -p "$HOME/ros_ws/underlay" "$HOME/ros_ws/underlay_build" "$HOME/ros_ws/underlay_install" \
  "$HOME/ros_ws/build" "$HOME/ros_ws/install" "$HOME/ros_ws/log"
```

## Expected Outcome
The ROS workspace directories will be created in the correct user home directory (e.g., `/home/user/ros_ws`) instead of failing to create `/ros_ws` due to permission errors.