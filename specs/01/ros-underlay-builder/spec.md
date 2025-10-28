# ROS Underlay Builder

## Goal
Automatically build a shared ROS underlay layer from repositories cloned by vcstool, with support for rebuilding inside the container without Docker rebuild.

## Requirements
- Discover `*.repos` and `depends.repos.yaml` files recursively like vcstool extension
- Build ROS underlay from all packages cloned by vcstool extension
- Install underlay to `/ros_underlay`
- Ensure `/ros_underlay` is writable by the build user before building (fixes colcon PermissionError)
- Source underlay in container environment
- Provide `build-underlay` command to rebuild underlay inside running container
- Support ROS workspace dependencies resolution with rosdep
- Use colcon for building packages

## Implementation
- Create `ros_underlay` extension that depends on `vcstool` and `ros_jazzy`
- Recursively find all `*.repos` files using `Path.cwd().rglob("*.repos")`
- Build packages from `/dependencies/<repo_parent_path>` directories cloned by vcstool
- Install to shared location `/ros_underlay`
- Add underlay to ROS environment paths (AMENT_PREFIX_PATH, etc.)
- Create executable `build-underlay` script for in-container rebuilds
- Use BuildKit cache mounts for faster builds
