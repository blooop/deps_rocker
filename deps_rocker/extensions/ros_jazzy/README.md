# ROS Jazzy Extension

The `ros_jazzy` extension provides a complete ROS 2 Jazzy development environment for Docker containers with unified workspace architecture, dynamic dependency management, and seamless integration with deps_rocker extensions.

## Quick Start

```bash
# Basic ROS Jazzy environment
deps_rocker --ros-jazzy ubuntu:24.04

# With additional development tools
deps_rocker --ros-jazzy --nvim --lazygit ubuntu:24.04
```

## Architecture Overview

The ros_jazzy extension implements a unified workspace architecture with two distinct layers:

- **Underlay Workspace** (`$HOME/underlay/`): Contains dependencies from `*.repos` files
- **Overlay Workspace** (`$HOME/overlay/`): Contains user packages (mounted via cwd extension)

Both workspaces use identical `src/build/install/log` structure and are managed by unified scripts.

## Implementation Details

### Base ROS Installation

Installs ROS 2 Jazzy foundation during Docker build:

- Adds official ROS 2 apt repository
- Installs `ros-jazzy-ros-core`, `python3-rosdep`, `python3-vcstool`
- Installs colcon via pip for build system
- Initializes rosdep database
- Sets base environment variables (`ROS_DISTRO=jazzy`)

### Dynamic Repository Management

**Key Design**: Repository management happens at **runtime**, not build time.

- No `*.repos` file processing during Docker build
- Provides `update_repos.sh` script for on-demand dependency import
- Changes to `.repos` files take effect immediately without Docker rebuild
- Eliminates repository staleness issues

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

**Workspace Creation:**
- Creates both underlay and overlay with identical `src/build/install/log` structure
- Installs unified management scripts (`workspace_deps.sh`, `workspace_build.sh`, `collect_packages.sh`, `update_repos.sh`)
- Configures shell to source ROS environments in proper order

**Package Collection:**
- Automatically discovers ROS packages in `$HOME` (from mounted directories)
- Creates symlinks in `$HOME/overlay/src/` pointing to user packages
- Enables seamless development with mounted source code

## Unified Script Architecture  

The extension provides a set of unified scripts that work identically on both underlay and overlay workspaces:

### Core Scripts

- **`workspace_deps.sh <workspace_path>`**: Installs rosdep dependencies for any workspace
- **`workspace_build.sh <workspace_path>`**: Builds any workspace using colcon  
- **`collect_packages.sh <workspace_path>`**: Collects user packages via symlinks into workspace
- **`update_repos.sh`**: Dynamically imports `*.repos` dependencies into underlay

### Legacy Compatibility Scripts

- **`underlay_deps.sh`**: Calls `workspace_deps.sh $HOME/underlay`
- **`underlay_build.sh`**: Calls `workspace_build.sh $HOME/underlay`  
- **`install_rosdeps.sh`**: Calls `workspace_deps.sh $HOME/overlay`

## Workspace Management Workflow

### Typical Development Flow

1. **Container Start**: Workspaces created, packages collected into overlay
2. **Dependency Management**: Run `update_repos.sh` to import dependencies to underlay  
3. **Build Process**: Use unified scripts or standard colcon commands
4. **Iterative Development**: Modify code, rebuild as needed

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

# Build workspaces using unified scripts
workspace_build.sh $HOME/underlay   # Build dependencies
workspace_build.sh $HOME/overlay    # Build user packages

# Or use standard colcon (from overlay workspace)
cd $HOME/overlay && colcon build
```

## Design Principles

1. **Unified Architecture**: Both underlay and overlay use identical workspace structure
2. **Runtime Discovery**: Repository dependencies processed dynamically, not at build time  
3. **Script Reusability**: Same scripts work for any workspace following the standard layout
4. **Clear Separation**: Dependencies (underlay) vs user packages (overlay)
5. **Permission Safety**: All workspaces in user home directory with proper ownership
6. **No Staleness**: Changes to `*.repos` files take effect immediately via `update_repos.sh`

## Usage Scenarios

### Basic Development
```bash
deps_rocker --ros-jazzy --cwd ~/my_project ubuntu:24.04
# Result: ~/my_project mounted and available in overlay workspace
```

### With Dependencies  
```bash
# If my_project contains *.repos files:
update_repos.sh                    # Import dependencies to underlay
workspace_build.sh $HOME/underlay  # Build dependencies  
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
- Essential tools: colcon, rosdep, vcstool
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
├── src/                # Symlinks to user packages from $HOME/package_name/
│   ├── package_name_1@ -> $HOME/package_name_1/
│   ├── package_name_2@ -> $HOME/package_name_2/
│   └── ...
├── build/              # Build artifacts
├── install/            # Install space
└── log/                # Build and test logs

$HOME/
├── package_name_1/     # User packages mounted via cwd extension
├── package_name_2/     # (e.g., ~/my_robot_pkg/, ~/demos/, etc.)
└── ...
```

### Workspace Layer Structure
1. **Base ROS**: `/opt/ros/jazzy/` (system installation)
2. **Underlay Layer**: `$HOME/underlay/install/` (dependency packages)
3. **Overlay Layer**: `$HOME/overlay/install/` (development packages from `$HOME/package_*/`)

## Extension Integration

### Dependencies
- **`curl`**: For downloading ROS repository keys
- **`git_clone`**: For repository operations  
- **`user`**: For proper user environment setup

### Related Extensions
- **`vcstool`**: Automatically included, provides `vcs import` functionality
- **`ros_underlay`**: Can build on top of ros_jazzy foundation
- **`cwd`**: Mounts user packages that get collected into overlay workspace

## Complete Usage Examples

### Basic Setup
```bash
# Create ROS Jazzy container with mounted project
deps_rocker --ros-jazzy --cwd ~/my_project ubuntu:24.04

# Inside container:
cd ~/overlay && colcon build  # User packages automatically available
```

### With Dependencies
```bash  
# Project has *.repos file with dependencies
deps_rocker --ros-jazzy --cwd ~/my_project ubuntu:24.04

# Inside container:
update_repos.sh              # Import dependencies to underlay
workspace_build.sh ~/underlay  # Build dependencies
cd ~/overlay && colcon build   # Build user packages
```

### Dynamic Repository Loading
```bash
renv ros2/demos              # Load any ROS repository
update_repos.sh              # Import its dependencies  
colcon build                 # Build everything
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

**`workspace_deps.sh <workspace_path>`**
- Installs rosdep dependencies for workspace packages
- Uses `rosdep install --from-paths <workspace>/src --ignore-src -y`

**`workspace_build.sh <workspace_path>`**  
- Builds workspace using `colcon build` with proper sourcing
- Works with any workspace following standard structure

**`collect_packages.sh <workspace_path>`**
- Discovers ROS packages in `$HOME` (excluding underlay/overlay dirs)
- Creates symlinks in `<workspace>/src/` pointing to user packages

**`update_repos.sh`**
- Scans workspace for `*.repos` files
- Consolidates and imports to `$HOME/underlay/src` using `vcs import`  
- Installs dependencies and builds underlay workspace
- Single command for complete dependency management

## Dynamic vs Static Repository Management

### Design Decision: Runtime Repository Discovery

**Problem with Build-Time Consolidation:**
- Changes to `*.repos` files require Docker image rebuild
- Repository state becomes "baked in" and stale
- Poor developer experience for iterative dependency changes

**Solution: Runtime Discovery:**
```bash
# Modify dependencies anytime
echo "navigation2:" >> my_deps.repos
echo "  type: git" >> my_deps.repos
echo "  url: https://github.com/ros-planning/navigation2.git" >> my_deps.repos

# Update immediately without Docker rebuild  
update_repos.sh
```

**Benefits:**
- Immediate effect of repository changes
- No Docker rebuild cycles
- Natural development workflow

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
├── collect_packages.sh             # Package collection script
├── update_repos.sh                 # Dynamic repository manager
├── auto_detect.yaml                # Auto-detection rules
└── configs/colcon-defaults.yaml    # Colcon configuration
```

### Testing Requirements
- Must work with any ROS repository (`ros2/demos`, arbitrary user projects)  
- Auto-detection must activate on `package.xml` and `*.repos` files
- Unified scripts must handle both underlay and overlay workspaces
- Dynamic repository management must work without Docker rebuilds

### Implementation Notes
- Remove build-time repository consolidation from `ros_jazzy.py`
- Implement unified scripts with identical behavior for both workspaces
- Ensure proper environment sourcing chain: base → underlay → overlay
- All workspace operations must work in user home directory with correct permissions