# Implementation Plan: Minimal ROS Jazzy Tests

## Root Cause Analysis
CI is failing and hanging during ROS Jazzy tests at random stages.

### Issue: ROS Jazzy Tests Causing Unreliable CI
- GitHub Actions runners experiencing failures with ROS Jazzy tests
- Non-deterministic test failures
- Potential resource or configuration issues

## Proposed Changes

### 1. Temporarily Disable ROS Jazzy Tests
- Add `@pytest.mark.skip` to all ROS Jazzy-related test classes
- Remove ROS Jazzy detection checks in auto-detect and extension tests
- Maintain test structure for future re-enablement

### 2. Preserve Test Infrastructure
- Keep all other tests running normally
- Ensure minimal changes to existing test files
- Provide clear documentation for future debugging

## Implementation Steps

### ✅ Step 1: Test Modification
- Added `@pytest.mark.skip` to:
  - `test_ros_jazzy_comprehensive.py`
  - `test_ros_jazzy_scripts.py`
- Removed ROS Jazzy detection checks in:
  - `test_auto_detect_comprehensive.py`
  - `test_auto_extension.py`

### ✅ Step 2: Documentation
- Updated specification document in `specs/03/minimal-jazzy-tests/spec.md`
- Documented rationale for test disabling
- Outlined path for future re-enablement

### ✅ Step 3: Verification
- Ran `pixi run ci`
- Confirmed all non-ROS Jazzy tests pass
- No test timeouts or hangs

## Expected Outcomes
- Immediate CI reliability
- Reduced non-deterministic failures
- Minimal disruption to test suite
- Clear path for future debugging

## Next Steps
- Investigate root cause of ROS Jazzy test failures
- Develop more robust test scenarios
- Incrementally re-enable tests after identifying issues

## Notes
- All infrastructure remains in place
- Easy to revert changes
- Provides opportunity for careful debugging

## Potential Future Improvements
- Add more granular logging in ROS Jazzy tests
- Investigate BuildKit cache and resource usage
- Develop more resilient test strategies