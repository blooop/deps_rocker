# Implementation Plan: Skip ROS Extension Tests

## Current State
- Some ROS tests already have skip decorators (`test_ros_jazzy_comprehensive.py`, `test_ros_jazzy_scripts.py`)
- Other ROS-related tests still run and cause slow CI/iteration times:
  - `test_ros_jazzy_extension` - already skipped
  - `test_ros_underlay_extension` - needs skip
  - `test_vcstool_extension` - needs skip
  - `test_auto_extension` - needs skip (depends on ROS)
  - `TestRosJazzyExtension` class in `test_ros_jazzy_robustness.py` - needs skip

## Files to Modify

### 1. `test/test_extensions_generic.py`
Add skip decorators to:
- `test_ros_underlay_extension` (line 242)
- `test_vcstool_extension` (line 245)
- `test_auto_extension` (line 248)

Skip reason: "Temporarily disabled - ROS tests are slow and impact iteration time"

### 2. `test/test_ros_jazzy_robustness.py`
Add class-level skip decorator to `TestRosJazzyExtension` (line 12)

Skip reason: "Temporarily disabled - ROS tests are slow and impact iteration time"

## Implementation Steps

1. Add `@pytest.mark.skip(reason="...")` decorator to each test method in test_extensions_generic.py
2. Add `@pytest.mark.skip(reason="...")` decorator to the TestRosJazzyExtension class
3. Run `pixi run ci` to verify all tests pass
4. Commit if CI passes

## Notes
- Use consistent skip message across all tests
- Tests can be easily re-enabled by removing the decorator
- This is a temporary measure for development velocity
