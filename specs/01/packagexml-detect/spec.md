# Fix package.xml Auto-Detection in Subdirectories

## Problem
The auto extension fails to detect `package.xml` files in subdirectories when using the `--auto` flag. This prevents automatic ROS extension activation for repositories with nested package structures like `ros2/demos`.

## Root Cause
The pattern matching uses `fnmatch.fnmatch()` which doesn't support glob-style recursive matching. The pattern `"package.xml"` only matches files at the root level, not in subdirectories like `"my_pkg/package.xml"`.

## Solution
Replace `fnmatch.fnmatch(f, pattern)` with `Path(f).match(pattern)` in the `_detect_glob_patterns` method. This enables proper glob pattern matching:
- Simple patterns like `"package.xml"` match files in any subdirectory
- Wildcard patterns like `"*.py"` continue to work
- Explicit recursive patterns like `"**/package.xml"` are supported

## Testing
- Added test for subdirectory detection: `test_detect_ros_in_subdirectory`
- Verified 20 `package.xml` files detected in `ros2/demos` repository
- All existing tests pass
