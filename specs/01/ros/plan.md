# Plan for Updating ROS Extension to Use apt_packages

1. Update spec.md to clarify apt_packages usage (done).
2. Refactor `deps_rocker/extensions/ros/ros.py` to use apt_packages for dependency installation.
3. Update `ros_snippet.Dockerfile` to delegate package installation to apt_packages.
4. Update or add tests to verify apt_packages is used for ROS dependencies.
5. Run `pixi run ci` and fix any errors until CI passes.
6. Commit only the relevant changes if CI passes.
