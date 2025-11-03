# ROS Jazzy Extension

The `ros_jazzy` extension provides a complete ROS 2 Jazzy development environment for Docker containers with unified workspace architecture, dynamic dependency management, and seamless integration with deps_rocker extensions.

## Quick Start

```bash
# Basic ROS Jazzy environment
rockerc --ros-jazzy ubuntu:24.04

# With additional development tools
rockerc --ros-jazzy --nvim --lazygit ubuntu:24.04
```

## Architecture Overview

The ros_jazzy extension implements a unified workspace architecture with two distinct layers:

- **Underlay Workspace** (`$HOME/underlay/`): Contains dependencies from `*.repos` files
- **Overlay Workspace** (`$HOME/overlay/`): Contains user packages (mounted automatically by the extension)

Both workspaces use identical `src/build/install/log` structure and are managed by unified scripts.

## Implementation Details

### Base ROS Installation

Installs ROS 2 Jazzy foundation during Docker build:

- Adds official ROS 2 apt repository
- Installs `ros-jazzy-ros-core`, `python3-rosdep`, `python3-vcs2l`
- Installs colcon via pip for build system
- Initializes rosdep database
- Sets base environment variables (`ROS_DISTRO=jazzy`)

### Hybrid Repository and Dependency Management

**Key Design**: Cached build-time dependencies with runtime update capability.

- **Docker Build**: Process `*.repos` files and install rosdep dependencies for both workspaces
- **Build Caching**: Build underlay workspace during Docker build (cached in layers)  
- **Runtime Flexibility**: Full capability to update underlay repos and re-run rosdep inside container
- **Persistent State**: Underlay workspace modifications persist within container lifetime
- **Best of Both**: Fast startup (cached) + full update capability (when needed)

### User Environment Setup

Creates unified workspace structure in user's home directory:

**Environment Variables:**
```bash
# Underlay workspace (dependencies)
ROS_UNDERLAY_ROOT=$HOME/underlay
ROS_UNDERLAY_PATH=$HOME/underlay/src
ROS_UNDERLAY_BUILD=$HOME/underlay/build  
ROS_UNDERLAY_INSTALL=$HOME/underlay/install

# Overlay workspace (user packages)
ROS_OVERLAY_ROOT=$HOME/overlay
ROS_WORKSPACE_ROOT=$HOME/overlay  # Main workspace
ROS_BUILD_BASE=$HOME/overlay/build
ROS_INSTALL_BASE=$HOME/overlay/install
ROS_LOG_BASE=$HOME/overlay/log
```

**Docker Build Process:**
- Discovers and consolidates `*.repos` files in build context
- Imports dependency repositories to underlay using `vcs import`
- Installs rosdep dependencies for both underlay and overlay (cached baseline)
- Builds underlay workspace with `colcon build` (cached in Docker layers)
- Creates overlay workspace structure ready for user packages
**Installs unified management scripts (`rosdep_underlay.sh`, `rosdep_overlay.sh`, `build_underlay.sh`, `update_repos.sh`)**
- **Preserves runtime capability**: Scripts can modify underlay and re-run rosdep inside container

**ROS-Specific Package Mounting:**
- Current working directory automatically mounted to `$HOME/overlay/src/$(basename $PWD)`
- Built-in mounting logic keeps workspace layout correct even when other extensions (e.g. `cwd`, `workdir`) are active
- No manual `--cwd` specification required - prevents mounting errors
- Workspace layout guaranteed to match ROS conventions
- `renv` containers automatically start in `$HOME/overlay/src`, matching the ROS workspace layout
- Enables seamless development with zero configuration

## Unified Script Architecture  

The extension provides a set of unified scripts that work identically on both underlay and overlay workspaces:

### Core Scripts

- **`rosdep_underlay.sh`**: Installs rosdep dependencies for underlay workspace
- **`rosdep_overlay.sh`**: Installs rosdep dependencies for overlay workspace  
- **`build_underlay.sh`**: Builds underlay workspace using colcon
- **`update_repos.sh`**: Dynamically imports `*.repos` dependencies into underlay



## Workspace Management Workflows

### Primary Workflow: Cached Baseline (90% of use cases)
1. **Docker Build**: 
   - Consolidate `*.repos` files and import to underlay
   - Install rosdep dependencies for underlay and overlay (baseline cached)
   - Build underlay workspace (cached in Docker layers)
2. **Container Start**: Underlay pre-built with cached dependencies
3. **Development**: Direct `colcon build` in overlay - dependencies ready
4. **Benefits**: Maximum caching, instant startup, predictable performance

### Update Workflow: Runtime Modifications (when needed)
1. **Container Running**: Start with cached baseline underlay
2. **Add Repositories**: Run `update_repos.sh` to import new `*.repos` files to underlay
3. **Update Dependencies**: Run `workspace_deps.sh ~/underlay` to install new rosdep packages
4. **Rebuild Underlay**: Run `workspace_build.sh ~/underlay` to build updated dependencies
5. **Continue Development**: Updated underlay available for overlay builds
6. **Persistence**: Changes persist for container lifetime, reset on container restart

### Hybrid Benefits
- **Fast startup**: Cached baseline means immediate productivity
- **Full flexibility**: Can add/modify any dependencies inside container
- **No limitations**: Complete rosdep and repository management capability
- **Development friendly**: Experiment with dependencies without Docker rebuilds

### Runtime Operations

**Environment Sourcing Chain:**
```bash
source /opt/ros/jazzy/setup.bash                    # Base ROS
source $HOME/underlay/install/setup.bash           # Dependencies (if built)
source $HOME/overlay/install/setup.bash            # User packages (if built)
```

**Build Commands:**
```bash
# Update dependencies from *.repos files
update_repos.sh

# Install underlay dependencies and build
cd ~/underlay && rosdep_underlay.sh && build_underlay.sh

# Install overlay dependencies and build
cd ~/overlay && rosdep_overlay.sh && colcon build --symlink-install
```

## Design Principles

1. **Unified Architecture**: Both underlay and overlay use identical workspace structure
2. **Hybrid Caching**: Build-time baseline + runtime update capability for best performance
3. **Script Reusability**: Same scripts work for build-time and runtime operations
4. **Clear Separation**: Dependencies (underlay) vs user packages (overlay)
5. **Permission Safety**: All workspaces in user home directory with proper ownership
6. **Full Runtime Capability**: Complete rosdep and repository management inside containers
7. **No Architectural Limitations**: Can perform any dependency operation at runtime

## Usage Scenarios

### Basic Development
```bash
cd ~/my_project && rockerc --ros-jazzy ubuntu:24.04
# Result: ~/my_project automatically mounted as ~/overlay/src/my_project
```

### With Dependencies  
```bash
cd ~/my_project && rockerc --ros-jazzy ubuntu:24.04
# Inside container - project auto-mounted, if it contains *.repos files:
update_repos.sh                    # Import dependencies to underlay
build_underlay.sh                  # Build dependencies  
cd $HOME/overlay && colcon build   # Build user packages
```

### Dynamic Repository Loading
```bash
renv ros2/demos  # Any repository with *.repos files
update_repos.sh  # Import dependencies dynamically
colcon build     # Build everything
```

## Features

## Key Features

### Complete ROS 2 Environment
- ROS 2 Jazzy from official apt repositories
- Essential tools: colcon, rosdep, vcs2l
- Automatic rosdep initialization
- Proper environment configuration

### Unified Workspace Architecture  
- **Identical Structure**: Both underlay and overlay use `src/build/install/log` layout
- **Clear Separation**: Dependencies (underlay) vs user packages (overlay)  
- **Unified Scripts**: Same management scripts work for both workspaces
- **User-Centric**: All workspaces in `$HOME` with proper permissions

### Dynamic Repository Management
- **Runtime Discovery**: Process `*.repos` files on-demand with `update_repos.sh`
- **No Staleness**: Repository changes take effect immediately
- **Generic Design**: Works with any ROS repository without hardcoding

### Auto-Detection
Automatically activates when ROS files are detected:
- `package.xml` files (ROS packages)
- `*.repos` files (repository manifests)
- ROS workspace indicators

## Workspace Structure

All workspaces are located in the user home directory for proper permission handling:

### Underlay Workspace (Dependencies)
```
$HOME/underlay/
├── src/                # Dependency source packages (from *.repos files)
├── build/              # Underlay build artifacts
├── install/            # Underlay install space
└── log/                # Underlay build logs
```

### Overlay Workspace (User Development)
```
$HOME/overlay/
├── src/                # User packages mounted directly here via cwd extension
│   ├── my_project/     # Auto-mounted: cd ~/my_project && rockerc --ros-jazzy
│   ├── another_pkg/    # (Extension determines mount path automatically)
│   └── ...
├── build/              # Build artifacts
├── install/            # Install space
└── log/                # Build and test logs
```

### Workspace Layer Structure
1. **Base ROS**: `/opt/ros/jazzy/` (system installation)
2. **Underlay Layer**: `$HOME/underlay/install/` (dependency packages)
3. **Overlay Layer**: `$HOME/overlay/install/` (user packages mounted directly in overlay/src/)

## Extension Integration

### Dependencies
- **`curl`**: For downloading ROS repository keys
- **`git_clone`**: For repository operations  
- **`user`**: For proper user environment setup

### Related Extensions
- **`vcstool`**: Automatically included, provides `vcs import` functionality via vcs2l
- **`ros_underlay`**: Can build on top of ros_jazzy foundation
- **ROS-specific mounting**: Extension handles workspace mounting automatically (no manual `--cwd` needed)

## Complete Usage Examples

### Standard Usage (Cached Dependencies)
```bash
# Mount current directory into overlay workspace automatically
cd ~/my_project && rockerc --ros-jazzy ubuntu:24.04

# Inside container - project automatically mounted in ~/overlay/src/my_project:
cd ~/overlay && colcon build  # Immediate build, dependencies already available
```

### Runtime Dependency Updates (Full Capability)
```bash  
# Start with cached baseline
cd ~/my_project && rockerc --ros-jazzy ubuntu:24.04

# Inside container - project auto-mounted, modify dependencies as needed:
update_repos.sh                 # Import new *.repos files to underlay
rosdep_underlay.sh              # Install new rosdep dependencies for underlay
build_underlay.sh               # Rebuild underlay with new packages
cd ~/overlay && colcon build    # Build overlay against updated underlay

# Or individual operations:
rosdep install --from-paths ~/underlay/src --ignore-src -y  # Direct rosdep for underlay
rosdep install --from-paths ~/overlay/src --ignore-src -y   # Direct rosdep for overlay
vcs import ~/underlay/src < my_new.repos                    # Direct vcs import
```

### Dynamic Repository Loading
```bash
renv ros2/demos              # Load any ROS repository (dependencies built during Docker build)
cd ~/overlay && colcon build # Build user packages - dependencies already cached
```

## Environment Variables

### Overlay Workspace Paths
- `ROS_OVERLAY_ROOT`: Overlay workspace root (`$HOME/overlay`)
- `ROS_WORKSPACE_ROOT`: Main workspace directory (points to overlay: `$HOME/overlay`)
- `ROS_BUILD_BASE`: Build artifacts directory (`$HOME/overlay/build`)
- `ROS_INSTALL_BASE`: Install space directory (`$HOME/overlay/install`)
- `ROS_LOG_BASE`: Log files directory (`$HOME/overlay/log`)

### Underlay Workspace Paths
- `ROS_UNDERLAY_ROOT`: Underlay workspace root (`$HOME/underlay`)
- `ROS_UNDERLAY_PATH`: Underlay source directory (`$HOME/underlay/src`)
- `ROS_UNDERLAY_BUILD`: Underlay build directory (`$HOME/underlay/build`)
- `ROS_UNDERLAY_INSTALL`: Underlay install directory (`$HOME/underlay/install`)

### ROS Configuration
- `ROS_DISTRO=jazzy`: ROS distribution identifier
- `COLCON_LOG_PATH`: Colcon log directory (uses workspace log path)

### Colcon Configuration
- **Default behavior**: Standard `colcon build`, `colcon test` commands work normally in both underlay and overlay
- **Automatic configuration**: Build arguments like `--symlink-install`, `--cmake-args` handled by colcon defaults
- **Log management**: Logs automatically directed to respective workspace log directories
- **Consistent workspace structure**: Both underlay (`$HOME/underlay/`) and overlay (`$HOME/overlay/`) use same layout

## Helper Scripts

Unified helper scripts work with any workspace following the consistent `src/build/install/log` structure.

### Script Specifications

**`rosdep_underlay.sh`**
- Installs rosdep dependencies for underlay packages
- Uses `rosdep install --from-paths ~/underlay/src --ignore-src -y`  
- **Runtime capable**: Can install new dependencies inside running container

**`rosdep_overlay.sh`**
- Installs rosdep dependencies for overlay packages  
- Uses `rosdep install --from-paths ~/overlay/src --ignore-src -y`
- **Runtime capable**: Can install dependencies for newly mounted packages

**`build_underlay.sh`**  
- Builds underlay workspace using `colcon build` with proper sourcing
- Works in `~/underlay` directory with correct environment setup
- **Overlay builds**: Always use `cd ~/overlay && colcon build` directly

**`update_repos.sh`**
- Scans workspace for `*.repos` files (including newly added ones)
- Imports to `$HOME/underlay/src` using `vcs import` (via vcs2l)
- Calls `rosdep_underlay.sh` to install dependencies for imported packages
- Calls `build_underlay.sh` to build updated underlay workspace
- **Runtime updates**: Complete underlay update workflow in single command

## Caching Strategy

### Design Decision: Build-Time Repository Processing with Runtime Flexibility

**Primary Approach: Docker Build Caching**
- Process `*.repos` files during Docker build for maximum caching
- Dependencies rarely change, so build-time processing is efficient
- Underlay workspace built and cached in Docker layers
- Fast container startup with pre-built dependencies

**Fallback: Runtime Updates When Needed**
```bash
# Only when adding new dependencies:
echo "navigation2:" >> my_deps.repos  
echo "  type: git" >> my_deps.repos
echo "  url: https://github.com/ros-planning/navigation2.git" >> my_deps.repos

# Update underlay incrementally
update_repos.sh  # Adds to existing cached underlay
```

**Benefits:**
- Maximum Docker layer caching for stable dependencies
- Fast container startup (dependencies pre-built)
- Incremental updates only when actually needed
- Best of both worlds: caching + flexibility

## Implementation Requirements

### Required Dependencies
- `curl`: For ROS repository setup
- `git_clone`: For repository operations  
- `user`: For proper user environment

### File Structure
```
deps_rocker/extensions/ros_jazzy/
├── ros_jazzy.py                    # Main extension class  
├── ros_jazzy_snippet.Dockerfile    # Base ROS installation
├── ros_jazzy_user_snippet.Dockerfile # User environment setup
├── workspace_deps.sh               # Unified dependency installer
├── workspace_build.sh              # Unified workspace builder  
├── update_repos.sh                 # Dynamic repository manager
├── auto_detect.yaml                # Auto-detection rules
└── configs/colcon-defaults.yaml    # Colcon configuration
```

### Testing Requirements
- Must work with any ROS repository (`ros2/demos`, arbitrary user projects)  
- Auto-detection must activate on `package.xml` and `*.repos` files
- Unified scripts must handle both underlay and overlay workspaces
- Runtime dependency updates must work inside running containers
- Both cached baseline and runtime modifications must function correctly
- Scripts must work identically during Docker build and container runtime

### Implementation Notes
- Keep build-time repository consolidation in `ros_jazzy.py` for optimal caching
- Build underlay workspace during Docker build with `colcon build`
- Implement unified scripts for runtime operations when needed
- **Implement ROS-specific mounting behavior:**
  - Automatically mount `$PWD` to `~/overlay/src/$(basename $PWD)` 
  - Override default cwd extension behavior with ROS workspace conventions
  - Prevent user mounting errors by enforcing correct structure
  - No `--cwd` flag needed - extension handles mounting automatically
- Ensure proper environment sourcing chain: base → underlay → overlay
- All workspace operations must work in user home directory with correct permissions
