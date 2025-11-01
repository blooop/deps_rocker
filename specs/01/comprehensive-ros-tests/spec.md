# ROS Jazzy Extension Comprehensive Test Suite

## Specification

Create a comprehensive test suite for the `ros_jazzy` extension that validates all features described in the README specification. The current implementation does not fully meet the specification, so tests will be implemented first, then the implementation will be updated to pass the tests.

## Key Features to Test

### 1. Base ROS Installation
- ROS 2 Jazzy from official repositories
- Essential tools: colcon, rosdep, vcstool
- Proper environment configuration
- All required ROS environment variables

### 2. Unified Workspace Architecture
- **Underlay Workspace** (`$HOME/underlay/`): Dependencies from `*.repos` files
- **Overlay Workspace** (`$HOME/overlay/`): User packages (auto-mounted via cwd)
- Both use identical `src/build/install/log` structure
- Proper environment variable setup for both workspaces

### 3. Hybrid Repository and Dependency Management  
- Build-time processing of `*.repos` files with caching
- Runtime update capability via unified scripts
- Best of both: fast startup + full flexibility

### 4. Unified Script Architecture
- `rosdep_underlay.sh`: Install rosdep deps for underlay
- `rosdep_overlay.sh`: Install rosdep deps for overlay  
- `build_underlay.sh`: Build underlay workspace
- `update_repos.sh`: Dynamic repository import

### 5. Auto-Detection
- Activates on `package.xml` files
- Activates on `*.repos` files
- ROS workspace indicators

### 6. ROS-Specific Package Mounting
- Auto-mount current directory to `$HOME/overlay/src/$(basename $PWD)`
- No manual `--cwd` specification required
- Enforces correct workspace structure

## Success Criteria
- All environment variables properly set according to specification
- Workspace structure matches specification exactly  
- All scripts work identically during build-time and runtime
- Auto-detection triggers correctly
- Package mounting works as specified
- Both cached baseline and runtime update workflows function
- Dependencies resolve correctly between underlay and overlay
- Full rosdep and repository management capability inside containers
