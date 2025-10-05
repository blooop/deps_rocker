# Update colcon defaults to use improved paths and options for ROS 2 builds
# This spec tracks the update of the colcon defaults in ros_generic/configs/defaults.yaml

- Use /home/ros_ws/ros_build for build, install, log, and test-result paths
- Add cmake-args for RelWithDebInfo and compile commands
- Add event-handlers for test
- Add clean.workspace section
- Add base-paths for build
- Add log-base for all commands
- Keep executor: multi-threaded if present
- Preserve any existing options unless overridden by the new defaults
