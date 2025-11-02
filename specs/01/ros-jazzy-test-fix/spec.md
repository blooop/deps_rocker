# Fix ros_jazzy extension failure

- Reproduce the failure with `pixi r test-extension ros_jazzy`.
- Resolve the colcon duplicate-package error caused by the smoke test fixture copying `test_package` twice.
- Ship a fix and validate with targeted and full CI runs.
