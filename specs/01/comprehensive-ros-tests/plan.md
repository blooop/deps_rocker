# ROS Jazzy Extension Comprehensive Test Plan

## Testing Strategy

This plan creates a comprehensive test suite that validates all features described in the ROS Jazzy extension README specification. Tests will be implemented first to define the expected behavior, then the implementation will be updated to pass all tests.

## Test Categories

### 1. Base ROS Installation Tests
**Purpose**: Verify core ROS 2 Jazzy installation and configuration
- Test ROS 2 commands are available (ros2, colcon, rosdep, vcs)
- Verify ROS environment variables are properly set
- Test ROS package discovery and basic functionality
- Verify official ROS repository setup

### 2. Environment Variable Tests  
**Purpose**: Validate all workspace environment variables match specification
**Required Variables**:
- `ROS_DISTRO=jazzy`
- `ROS_UNDERLAY_ROOT=$HOME/underlay`
- `ROS_UNDERLAY_PATH=$HOME/underlay/src`
- `ROS_UNDERLAY_BUILD=$HOME/underlay/build`
- `ROS_UNDERLAY_INSTALL=$HOME/underlay/install`
- `ROS_OVERLAY_ROOT=$HOME/overlay`
- `ROS_WORKSPACE_ROOT=$HOME/overlay` (Main workspace)
- `ROS_BUILD_BASE=$HOME/overlay/build`
- `ROS_INSTALL_BASE=$HOME/overlay/install`
- `ROS_LOG_BASE=$HOME/overlay/log`

### 3. Workspace Structure Tests
**Purpose**: Verify both workspaces have correct directory structure
**Underlay Structure**:
```
$HOME/underlay/
├── src/        (dependency packages from *.repos)
├── build/      (underlay build artifacts)
├── install/    (underlay install space)
└── log/        (underlay build logs)
```

**Overlay Structure**:
```
$HOME/overlay/
├── src/        (user packages, auto-mounted)
├── build/      (build artifacts)
├── install/    (install space)
└── log/        (build and test logs)
```

### 4. Unified Script Tests
**Purpose**: Verify all management scripts work correctly
**Scripts to Test**:
- `rosdep_underlay.sh`: Install rosdep deps for underlay
- `rosdep_overlay.sh`: Install rosdep deps for overlay
- `build_underlay.sh`: Build underlay workspace with colcon
- `update_repos.sh`: Dynamic repository import to underlay

**Test Requirements**:
- Scripts work identically during Docker build and container runtime
- Scripts handle both empty and populated workspaces correctly
- Scripts maintain proper environment sourcing
- Error handling and logging work correctly

### 5. Repository Management Tests
**Purpose**: Verify hybrid repository and dependency management
**Build-time Caching Tests**:
- Process `*.repos` files during Docker build
- Import dependencies to underlay workspace
- Build underlay during Docker build (cached in layers)
- Verify fast container startup with pre-built dependencies

**Runtime Update Tests**:
- Test `update_repos.sh` can import new repositories
- Verify rosdep can install new dependencies
- Test underlay can be rebuilt inside running container  
- Validate changes persist within container lifetime

### 6. Auto-Detection Tests
**Purpose**: Verify extension automatically activates correctly
**Detection Triggers**:
- Presence of `package.xml` files (ROS packages)
- Presence of `*.repos` files (repository manifests)
- ROS workspace indicators

### 7. Package Mounting Tests  
**Purpose**: Verify ROS-specific automatic mounting behavior
**Requirements**:
- Current directory auto-mounts to `$HOME/overlay/src/$(basename $PWD)`
- No manual `--cwd` specification required
- Mounting prevents user errors and enforces ROS conventions
- Works seamlessly with zero configuration

### 8. Dependency Resolution Tests
**Purpose**: Verify packages can depend on underlay and resolve correctly
**Scenarios**:
- User packages in overlay depend on underlay packages
- rosdep resolves dependencies for both workspaces
- Environment sourcing chain works: base → underlay → overlay
- colcon builds work with proper dependency resolution

### 9. Workflow Tests
**Purpose**: Verify both primary and update workflows work
**Primary Workflow (90% use case)**:
1. Docker build processes `*.repos` and builds underlay
2. Container starts with pre-built underlay (cached)
3. User develops directly in overlay with dependencies ready

**Update Workflow (when needed)**:
1. Container starts with cached baseline
2. Add new repositories with `update_repos.sh`
3. Install new dependencies with `rosdep_underlay.sh`
4. Rebuild underlay with `build_underlay.sh`
5. Continue development with updated underlay

### 10. Permission and User Tests
**Purpose**: Verify proper file ownership and multi-user support
**Requirements**:
- All workspaces in user home directory with correct ownership
- Non-root users can read/write all workspace directories
- Scripts work for any user, not just root
- Proper handling of Docker user mapping

## Success Metrics

### Functional Requirements
- ✅ All environment variables match specification exactly
- ✅ Workspace structure created correctly in both scenarios  
- ✅ All unified scripts work during build-time and runtime
- ✅ Auto-detection triggers on correct file patterns
- ✅ Package mounting follows ROS conventions automatically
- ✅ Both cached and update workflows complete successfully
- ✅ Dependencies resolve between underlay and overlay correctly

### Performance Requirements  
- ✅ Container startup < 5 seconds with cached underlay
- ✅ Underlay builds cache properly (no repeated work)
- ✅ Runtime updates don't rebuild entire underlay unnecessarily
- ✅ BuildKit cache mounts speed up apt operations

### Robustness Requirements
- ✅ Handle missing or empty `*.repos` files gracefully
- ✅ Work correctly with any ROS repository (ros2/demos, user projects)
- ✅ Proper error messages and logging throughout
- ✅ File permissions work for all users
