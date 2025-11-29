## Steps
- Inspect foxglove extension (Docker snippet, wrapper, tests) and CI failure logs to pinpoint why CI needs `xauth`.
- Ensure x11 dependency handling installs required tooling (likely `xauth`) or adjusts wrapper/test to avoid missing binary in headless CI.
- Update foxglove assets accordingly and align tests with CI behavior.
- Run `pixi run ci`; iterate until tests pass.
