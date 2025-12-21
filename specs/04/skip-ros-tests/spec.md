# Temporarily Skip ROS Extension Tests

## Objective
Disable all ROS-related extension tests to improve iteration time during development.

## Scope
Add `@pytest.mark.skip` decorators to all ROS extension tests:
- `test_ros_underlay_extension`
- `test_vcstool_extension`
- `test_auto_extension`
- `TestRosJazzyExtension` class tests
- ROS auto-detection tests (package.xml detection)

## Reason
ROS tests are slow (requiring full ROS environment builds) and significantly impact development iteration speed. These will be re-enabled once performance optimizations are in place.

## Duration
Temporary - to be reversed when ROS test performance is improved and ROS auto-detection is re-enabled.
