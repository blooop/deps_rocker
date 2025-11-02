# Remove symlink requirement in ros_jazzy tests

- Update ros_jazzy extension tests so they no longer rely on creating a symbolic link in the workspace.
- Copy the ROS fixture directly into `src/test_package` (or equivalent) so the smoke test still validates the package build.
- Ensure the updated approach still validates the extension behaviour and passes `pixi run ci`.
