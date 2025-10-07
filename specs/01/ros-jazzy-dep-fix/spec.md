# ROS Jazzy dependency fixes

## Summary
- Reference `vcstool` correctly in `ros_jazzy.depends_on_extensions`.
- Ensure `ros_underlay` declares a dependency on `ros_jazzy` so the ROS environment is ready.
- Consolidate the underlay build steps to a single source to prevent drift.
- Fix the `pixi run ci` task definition so it conforms to the allowed keys.
- Deliver a passing `pixi run ci`.

## Acceptance
- `pixi run ci` exits successfully.
- `ros_jazzy` resolves dependencies without typos.
- `ros_underlay` initializes after ROS Jazzy setup.
- Underlay build logic is authored once and reused elsewhere.
