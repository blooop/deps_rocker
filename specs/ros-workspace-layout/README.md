# ROS Workspace Layout

- Adopt `/workspaces/ros_ws` as the canonical root with `repos/`, `src/`, and `underlay/` subdirectories and publish it via `ROS_WORKSPACE_ROOT` and friends.
- Route vcstool imports into `src/` while preserving manifest copies in `repos/`, then build/install under `underlay/` so the user owns the artifacts.
- Ensure ros_jazzy, ros_underlay, vcstool, and helper scripts share the layout defaults and remain easy to override when needed.
- Make underlay tooling exit cleanly when ROS base setup scripts are absent (e.g. unit tests or custom builds).
- Propagate build/log directory env vars (e.g. COLCON_LOG_PATH) so `colcon` CLIs use the writable workspace paths by default.

# Plan

1. Audit current ros_jazzy, vcstool, ros_underlay, and cwd extensions to map where repos and underlays live now.
2. Evaluate candidate directory layouts (root, repo workspace, home) against permission model and UX.
3. Select canonical layout that keeps artifacts writable and discoverable without polluting downstream repos.
4. Design required code changes across extensions (env vars, Docker snippets, helper scripts) to adopt the layout.
5. Outline migration steps (cleanup paths, symlinks, documentation) to ease transition.
