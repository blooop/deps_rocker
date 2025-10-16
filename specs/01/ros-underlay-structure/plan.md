# Implementation Plan

## 1. Update Environment Variables (ros_jazzy_snippet.Dockerfile)
- Change `ROS_UNDERLAY_PATH` from `/ros_ws/underlay` to `/ros_ws/underlay/src`
- Change `ROS_UNDERLAY_BUILD` from `/ros_ws/underlay_build` to `/ros_ws/underlay/build`
- Change `ROS_UNDERLAY_INSTALL` from `/ros_ws/underlay_install` to `/ros_ws/underlay/install`
- Add `ROS_UNDERLAY_LOG=/ros_ws/underlay/log`
- Update mkdir commands to create new directory structure

## 2. Update User Layer (ros_jazzy_user_snippet.Dockerfile)
- Update VCS import to target `ROS_UNDERLAY_PATH` (which is now /ros_ws/underlay/src)
- Cache path should be `/underlay` (without src, since vcs import structure is flat)
- Copy destination should be `$ROS_UNDERLAY_PATH` (=/ros_ws/underlay/src)

## 3. Update Build Scripts
- underlay_build.sh: Update defaults to new paths
- underlay_deps.sh: Update defaults to new paths

## 4. Update Unit Tests (test_ros_jazzy_scripts.py)
- Update path construction to match new structure
- Change `underlay_path` to `underlay_path / "src"` where appropriate
- Update directory assertions

## 5. Testing
- Run `pixi r test-extension ros_jazzy` (~15 min)
- Verify all tests pass
- Run `pixi r fix-commit-push`
