# Deprecate isaac_sim and convert ROS to robotstack (disk space optimization)

CI is failing on GitHub when testing all extensions together due to disk space exhaustion.

## Actions

1. **Deprecate isaac_sim extension** - Not widely used, complex dependencies causing disk space issues
2. **Convert ROS Jazzy to use robotstack via pixi** - Eliminate massive apt dependencies
3. **Investigate disk space usage** - Identify what's causing the space issues

## Potential causes of disk space issues:
- Docker layer caching with BuildKit (112GB build cache, 81GB reclaimable)
- 16+ unique cache mount IDs across all extensions
- Multiple large builder stages running in parallel
- Pixi cache growing during builds
- Docker images not being cleaned up properly (129.6GB images, 101.4GB reclaimable)
- **ROS apt packages are extremely large** - tens of GB for full desktop install

## Changes

### isaac_sim
1. Convert isaac_sim extension to a deprecation stub
2. Show deprecation warning when extension is used
3. Remove all functionality to eliminate builder stages and dependencies
4. Keep entry point for backward compatibility

### ros_jazzy
1. Replace apt packages with robotstack pixi packages
2. Use `ros-jazzy-desktop` from conda-forge/robotstack
3. Remove apt repository setup and key management
4. Simplify Dockerfile significantly (from 64 lines to 35 lines)
5. Maintain vcstool and colcon functionality
6. Significantly reduce disk space usage:
   - No apt repository cache
   - No apt package cache
   - Conda packages more efficiently compressed
   - Pixi cache more efficient than apt

## Results
- All CI tests passing (76 passed, 13 skipped)
- Significantly reduced disk space usage
- Faster ROS installation (no apt update/upgrade needed)
- Cleaner, more maintainable code
