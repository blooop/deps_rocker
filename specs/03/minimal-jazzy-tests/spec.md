# Spec: Minimal ROS Jazzy Tests

## Problem
CI is hanging during ROS Jazzy tests at random stages. Root causes:
1. **Resource exhaustion** on GitHub Actions runners (2 cores, 7GB RAM)
2. **Systemd services starting** during apt package installation in Docker containers
3. **dpkg lock contention** when multiple processes try to use apt simultaneously

## Solution
Two-pronged approach:

### A. Skip Resource-Intensive Tests in CI
1. Add `@pytest.mark.slow` decorator to comprehensive ROS Jazzy tests
2. Configure CI coverage task to skip slow tests with `-m 'not slow'`
3. Provide `test-all` and `coverage-all` tasks for running full test suite locally
4. Remove large `geometry2` repository from remaining test fixtures

### B. Fix Docker Container Service Hang Issues
1. Prevent systemd services from starting during package installation using `policy-rc.d`
2. Configure apt to wait for dpkg locks with timeout instead of failing immediately
3. Add retry logic for apt operations

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
