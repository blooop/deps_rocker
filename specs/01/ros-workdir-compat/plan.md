1. Audit current `ros_jazzy` workdir override: inspect extension wiring, Docker snippets, and related scripts.
2. Decide approach: either remove the dedicated workdir extension or adapt it so other extensions still set their paths correctly.
3. Update code and Docker snippets to implement the chosen fix while keeping ROS functionality intact.
4. Refresh tests (unit + shell) so `pixi run ci` validates the new behavior.
5. Document any usage changes in extension docstrings or README if required.
