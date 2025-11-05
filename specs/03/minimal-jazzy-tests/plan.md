# Implementation Plan: Minimal ROS Jazzy Tests

## Current Situation
The `test_ros_jazzy_comprehensive.py` test suite uses two repositories:
1. `unique_identifier_msgs` - Small, header-only package (~100KB)
2. `geometry2` - Large package collection with tf2, tf2_ros, etc. (~several MB, many dependencies)

The `geometry2` repository is causing CI hangs due to:
- Large clone size
- Multiple packages requiring rosdep installs
- Long colcon build times
- Heavy resource usage on CI runners

## Proposed Changes

### Test Files to Modify

#### `test/test_ros_jazzy_comprehensive.py`
**Line 109-116** - `test_ros_jazzy_with_dependencies_repos`:
- **Current**: Uses both `unique_identifier_msgs` and `geometry2`
- **Change**: Remove `geometry2`, keep only `unique_identifier_msgs`
- **Rationale**: This test validates multi-repo imports, rosdep, and builds. A single small repo is sufficient.

**Alternative approach**: If multi-repository testing is critical, replace `geometry2` with another minimal package like:
- `rcl_interfaces` (ROS 2 core interfaces, small)
- `common_interfaces` subset (std_msgs, std_srvs - though may already be installed)

### Files That Don't Need Changes

- `test_ros_jazzy_comprehensive.py` line 164-167: Already uses only `unique_identifier_msgs` ✓
- `test_ros_jazzy_robustness.py`: Uses minimal test fixtures ✓
- `test_ros_jazzy_scripts.py`: Doesn't use external repos ✓
- `deps_rocker/extensions/ros_jazzy/test.sh`: Uses apt packages only ✓

## Implementation Steps

1. **Update test fixtures** in `test_ros_jazzy_comprehensive.py`:
   - Remove `geometry2` from the repositories YAML
   - Keep `unique_identifier_msgs` as the sole test repository

2. **Verify test coverage**:
   - Ensure all critical scenarios still pass:
     - Repository cloning
     - Rosdep dependency installation
     - Colcon build process
     - Script execution (colcon_build.sh, etc.)

3. **Run CI locally**:
   - Execute `pixi run ci` to verify changes
   - Monitor build times and resource usage

4. **Iterate if needed**:
   - If tests fail, add back minimal second repository
   - Ensure no test logic depends on geometry2-specific features

## Expected Outcomes

### Before
- CI hangs at various stages during geometry2 clone/build
- Test runtime: 5-10+ minutes (when it doesn't timeout)
- High resource usage on CI runners

### After
- Consistent CI completion without hangs
- Test runtime: 2-3 minutes
- Minimal resource usage
- All test scenarios still validated

## Testing Checklist

- [ ] `test_ros_jazzy_with_dependencies_repos` passes with single repository
- [ ] `test_ros_jazzy_in_container_scripts_functionality` passes (already minimal)
- [ ] All other ROS Jazzy tests pass
- [ ] CI completes without timeouts
- [ ] Test coverage remains comprehensive

## Rollback Plan

If removing `geometry2` breaks critical test scenarios:
1. Identify what geometry2 was specifically testing
2. Find minimal alternative that tests the same functionality
3. Use smaller ROS 2 packages like `rcl_interfaces` or `example_interfaces`

## Notes

- `unique_identifier_msgs` is ideal for testing because:
  - Small size (~100KB)
  - Minimal dependencies (only std_msgs)
  - Header-only package (fast builds)
  - Available in ROS 2 Jazzy
  - Sufficient to validate rosdep, vcs import, and colcon build workflows
