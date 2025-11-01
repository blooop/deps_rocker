## Goal
- Allow `ros_jazzy` to run alongside other extensions without a conflicting workdir override.

## Constraints
- Keep workdir behavior predictable for users invoking `renv`.
- Preserve ROS Jazzy usability and existing test coverage.

## Deliverables
- `ros_jazzy` handles its own mount/working-dir setup without relying on the separate `workdir` extension.
- `pixi run ci` succeeds.
