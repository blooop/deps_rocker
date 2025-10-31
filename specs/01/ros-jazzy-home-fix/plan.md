# ROS Jazzy $HOME Variable Fix Implementation Plan

## Current Issue
The `ros_jazzy_user_snippet.Dockerfile` fails with permission errors because it uses `$HOME` environment variable which is not available during Docker build time.

## Steps to Fix

### 1. Update ROS Jazzy Extension Python Code
- Add import for `pwd` module to get user info
- Override the `empy_user_args` property to include the container home directory
- Get home directory using `cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir`

### 2. Convert User Snippet to EmPy Template
- Replace `$HOME` references with `@(container_home)` EmPy template variables
- This will be expanded during template processing with the actual path

### 3. Update Environment Variables
Instead of:
```dockerfile
ENV ROS_WORKSPACE_ROOT=$HOME/ros_ws
```
Use:
```dockerfile
ENV ROS_WORKSPACE_ROOT=@(container_home)/ros_ws
```

### 4. Update RUN Commands
Replace all `$HOME` references in RUN commands with the `@(container_home)` template variable.

### 5. Test the Fix
- Run the extension build to verify no permission errors
- Ensure directories are created in the correct location
- Verify all bashrc modifications work correctly

## Files to Modify
1. `deps_rocker/extensions/ros_jazzy/ros_jazzy.py` - Add empy_user_args with container_home
2. `deps_rocker/extensions/ros_jazzy/ros_jazzy_user_snippet.Dockerfile` - Replace $HOME with @(container_home)

## Validation
- Build should complete without permission errors
- ROS workspace should be created in user home directory
- Environment variables should point to correct paths