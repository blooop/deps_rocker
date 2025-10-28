# Comprehensive ROS Test Suite with Performance Optimizations

## Problem
ROS tests are very slow (dominated by apt installs) and `underlay_build.sh` fails with permissions issues when run inside containers. Need comprehensive tests that verify all ROS features work correctly.

## Solution
1. Fix permissions issues in underlay scripts
2. Speed up apt installs using BuildKit cache mounts
3. Create comprehensive test suite covering:
   - ROS environment setup
   - Underlay workspace build
   - VCS tool integration
   - Package building with dependencies
   - Multi-workspace scenarios

## Key Changes
- Use apt cache mounts in Dockerfile to speed up repeated installs
- Fix permissions in `underlay_build.sh` and `underlay_deps.sh`
- Add comprehensive test script that validates all ROS functionality
- Test both with and without underlay dependencies
