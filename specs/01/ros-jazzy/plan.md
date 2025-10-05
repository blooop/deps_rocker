# Plan for ROS Jazzy Extension

1. Copy ros_generic extension directory to ros_jazzy
2. Update __init__.py to import ros_jazzy
3. Create ros_jazzy.py:
   - Inherit from SimpleRockerExtension
   - Set name = "ros_jazzy"
   - Update docstring for ROS 2 Jazzy
   - Adjust dependencies if needed
4. Create ros_jazzy_snippet.Dockerfile:
   - Update install commands for ROS 2 Jazzy
   - Use correct ROS 2 Jazzy apt sources and keys
5. Create test.sh:
   - Check ros2 command is available
   - Test basic ros2 functionality
6. Update pyproject.toml entry points
7. Add to EXTENSIONS_TO_TEST in test_extensions_generic.py
8. Add test method: test_ros_jazzy_extension
9. (Optional) Update README.md if needed
10. Commit only the contents of specs/01/ros-jazzy/
