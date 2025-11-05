# Spec: Minimal ROS Jazzy Tests

## Problem
CI is hanging during ROS Jazzy tests at various stages, particularly when cloning and building large repositories like `geometry2`. This causes test timeouts and unreliable CI runs.

## Solution
Reduce test repository size by:
1. Remove large repositories like `geometry2` from test fixtures
2. Use only minimal repositories (e.g., `unique_identifier_msgs`) that prove functionality
3. Keep all critical test scenarios but with lighter dependencies

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
