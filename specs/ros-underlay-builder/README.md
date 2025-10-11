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

# ROS Underlay Builder - Implementation Plan

## 1. Extension Structure
Create `deps_rocker/extensions/ros_underlay/` with:
- `__init__.py` - Export extension class
- `ros_underlay.py` - Main extension class
- `ros_underlay_snippet.Dockerfile` - Docker build commands
- `build-underlay.sh` - Rebuild script for container

## 2. Extension Implementation (`ros_underlay.py`)
- Inherit from `SimpleRockerExtension`
- Set name = `"ros_underlay"`
- Depend on `("vcstool", "ros_jazzy")`
- No additional CLI arguments needed

## 3. Dockerfile Snippet (`ros_underlay_snippet.Dockerfile`)
- Run rosdep update
- Install dependencies from /dependencies with rosdep
- Build packages with colcon to /ros_underlay
- Use BuildKit cache mounts for colcon build cache
- Set environment variables to include underlay paths
- Copy build-underlay script to /usr/local/bin

## 4. Build Script (`build-underlay.sh`)
- Source ROS setup
- Run rosdep install for /dependencies
- Build with colcon to /ros_underlay
- Make executable and copy to /usr/local/bin during Docker build

## 5. Entry Point
Add to pyproject.toml:
```toml
ros_underlay = "deps_rocker.extensions.ros_underlay.ros_underlay:RosUnderlay"
```

## 6. Testing
- Add to `EXTENSIONS_TO_TEST` in test_extensions_generic.py
- Create test.sh to verify:
  - Underlay exists at /ros_underlay
  - build-underlay command exists and is executable
  - Environment variables are set correctly

## 7. Environment Setup
Export these in Dockerfile:
- AMENT_PREFIX_PATH prepend /opt/ros_underlay
- COLCON_PREFIX_PATH prepend /opt/ros_underlay
- LD_LIBRARY_PATH prepend /opt/ros_underlay/lib
- PATH prepend /opt/ros_underlay/bin
- PYTHONPATH prepend /opt/ros_underlay/lib/python*/site-packages
