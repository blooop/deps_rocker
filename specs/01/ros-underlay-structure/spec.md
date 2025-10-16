# ROS Underlay Structure Update

## Goal
Update ROS workspace structure to use a cleaner layout: `/ros_ws/underlay/{src,build,install,log}` instead of separate `/ros_ws/{underlay,underlay_build,underlay_install}` directories.

## Changes
- `ROS_UNDERLAY_PATH` → `/ros_ws/underlay/src` (source files)
- `ROS_UNDERLAY_BUILD` → `/ros_ws/underlay/build` (build artifacts)
- `ROS_UNDERLAY_INSTALL` → `/ros_ws/underlay/install` (install prefix)
- Add `ROS_UNDERLAY_LOG` → `/ros_ws/underlay/log` (build logs)

## Affected Components
- Environment variables in Dockerfile
- Build scripts (underlay_build.sh, underlay_deps.sh)
- User layer VCS import
- Unit tests

## Performance Optimization
Replace slow `chmod -R 777` operations with `install -d -m 777` to set permissions at directory creation time, avoiding recursive permission changes.
