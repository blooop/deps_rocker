# Implementation Plan: Comprehensive ROS Tests with Performance Optimizations

## Analysis of Current Issues

### Permissions Problem
The `underlay_build.sh` script runs as root during Docker build but the directories need to be accessible by non-root users at runtime. Currently only setting 777 on workspace directories but build/install dirs may have restrictive perms from colcon.

### Performance Issue
The Dockerfile currently uses `--mount=type=cache` for apt and pip, but:
1. The apt cache mount is already in place (line 6, 62)
2. Rosdep install is slow because it fetches package lists every time
3. VCS import cache is working well

## Implementation Steps

### 1. Fix Permissions Issues
- Ensure underlay build/install directories have proper permissions (a+rwX)
- Add chmod after underlay build completes
- Test that non-root users can source and use the underlay

### 2. Optimize Build Performance
- Verify apt cache mounts are working correctly
- Add rosdep cache if possible
- Consider using --skip-keys for known packages
- Profile build times to identify bottlenecks

### 3. Create Comprehensive Test Suite
The test.sh script should verify:
- Basic ROS installation (ros2 command, environment vars)
- Colcon and vcstool availability
- Underlay workspace:
  - Dependencies installed correctly
  - Packages built successfully
  - Can be sourced
  - Packages available in ROS path
- Main workspace:
  - Can build packages that depend on underlay
  - Proper workspace chaining
  - Environment isolation
- Multiple test scenarios:
  - With underlay dependencies
  - Without underlay (empty depends.repos)
  - Building against underlay

### 4. Test Matrix
Create tests for:
- `ros_jazzy` extension alone
- `ros_jazzy` + `vcstool` (implicit via depends.repos)
- `ros_underlay` extension
- Full integration with actual package builds

### 5. Performance Metrics
- Measure baseline build time
- Measure optimized build time
- Document improvements in spec

## Files to Modify
1. `deps_rocker/extensions/ros_jazzy/ros_jazzy_snippet.Dockerfile` - ensure cache mounts optimal
2. `deps_rocker/extensions/ros_jazzy/underlay_build.sh` - fix permissions
3. `deps_rocker/extensions/ros_jazzy/underlay_deps.sh` - optimize rosdep
4. `deps_rocker/extensions/ros_jazzy/test.sh` - enhance test coverage
5. `test/test_extensions_generic.py` - potentially add separate ROS test class

## Success Criteria
- All ROS tests pass
- underlay_build.sh works in container
- Build time reduced by at least 20%
- Comprehensive test coverage of all ROS features
