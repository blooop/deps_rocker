## Goal
- Allow `ros_jazzy` to run alongside other extensions without a conflicting workdir override.

## Constraints
- Keep workdir behavior predictable for users invoking `renv`.
- Preserve ROS Jazzy usability and existing test coverage.

## Deliverables
- `ros_jazzy` no longer depends on the bespoke workdir extension, or that extension becomes compatible with others.
- `pixi run ci` succeeds.
