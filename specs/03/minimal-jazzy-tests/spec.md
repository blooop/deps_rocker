# Spec: Minimal ROS Jazzy Tests

## Problem
CI is hanging during ROS Jazzy tests at random stages due to resource exhaustion on GitHub Actions runners. This causes test timeouts and unreliable CI runs.

## Solution
Skip resource-intensive comprehensive tests in CI while keeping them available for local testing:
1. Add `@pytest.mark.slow` decorator to comprehensive ROS Jazzy tests
2. Configure CI coverage task to skip slow tests with `-m 'not slow'`
3. Provide `test-all` and `coverage-all` tasks for running full test suite locally
4. Remove large `geometry2` repository from remaining test fixtures

## Implementation Details
- Update `test_ros_jazzy_comprehensive.py` to remove `geometry2` from test cases
- Retain `unique_identifier_msgs` as the primary test repository (small, header-only, minimal dependencies)
- Ensure all test scenarios still validate:
  - Multi-repository imports
  - Rosdep installation
  - Colcon builds
  - Script functionality

## Benefits
- Faster CI execution (avoid large repository clones/builds)
- More reliable tests (less prone to timeouts and resource exhaustion)
- Reduced GitHub Actions runner costs
- Maintains full test coverage with minimal resources
