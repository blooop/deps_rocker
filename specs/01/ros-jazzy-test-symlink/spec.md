# Remove symlink requirement in ros_jazzy tests

- Update ros_jazzy extension tests so they no longer rely on creating a symbolic link in the workspace.
- Ensure the updated approach still validates the extension behaviour and passes `pixi run ci`.
