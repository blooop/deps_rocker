# ROS Jazzy dependency fixes

## Summary
- Reference `vcstool` correctly via the `ros_jazzy.depends_on_extension` tuple.
- Ensure `ros_underlay` declares a dependency on `ros_jazzy` so the ROS environment is ready.
- Consolidate the underlay build steps to a single source to prevent drift.
- Fix the `pixi run ci` task definition so it conforms to the allowed keys (move per-extension testing behind a wrapper script).
- Deliver a passing `pixi run ci`.

## Acceptance
- `pixi run ci` exits successfully.
- `ros_jazzy` resolves dependencies without typos.
- `ros_underlay` initializes after ROS Jazzy setup.
- Underlay build logic is authored once and reused elsewhere.

# Plan

1. Review `ros_jazzy` and `ros_underlay` extensions along with their Docker snippets and scripts to map existing dependencies and build flow.
2. Update the `ros_jazzy.depends_on_extension` tuple to spell `vcstool` correctly.
3. Amend `ros_underlay.depends_on_extension` to include `ros_jazzy` in addition to the tooling it already requires.
4. Decide on the primary home for the underlay build commands (likely the shell script), refactor the Docker snippet or script so the commands live in just one spot, and ensure both paths invoke the shared logic.
5. Fix the `pixi` CI task definition in `pyproject.toml` so it only uses supported keys and still exercises the extension tests.
6. Run `pixi run ci`, address any failures, and repeat until green.
7. Prepare the change set for review and commit after CI passes.
