# ROS Workspace Layout

- Adopt `/ros_ws/ros_ws` as the canonical root with `repos/`, `src/`, and `underlay/` subdirectories and publish it via `ROS_WORKSPACE_ROOT` and friends.
- Route vcstool imports into `src/` while preserving manifest copies in `repos/`, then build/install under `underlay/` so the user owns the artifacts.
- Ensure ros_jazzy, ros_underlay, vcstool, and helper scripts share the layout defaults and remain easy to override when needed.
- Make underlay tooling exit cleanly when ROS base setup scripts are absent (e.g. unit tests or custom builds).
- Propagate build/log directory env vars (e.g. COLCON_LOG_PATH) so `colcon` CLIs use the writable workspace paths by default.
