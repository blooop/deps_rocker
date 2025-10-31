# ROS Jazzy Extension

The `ros_jazzy` extension provides a complete ROS 2 Jazzy Jalopy development environment for Docker containers, with proper workspace layout, dependency management, and integration with other deps_rocker extensions.

## Quick Start

```bash
# Basic ROS Jazzy environment
deps_rocker --ros-jazzy ubuntu:24.04

# With additional development tools
deps_rocker --ros-jazzy --nvim --lazygit ubuntu:24.04
```

## Workflow - How the Image is Built

This section describes the complete Docker image build process when using the ros_jazzy extension. Understanding this workflow is crucial for troubleshooting and customization.

### Phase 1: Base ROS Installation (`ros_jazzy_snippet.Dockerfile`)

The main Dockerfile snippet handles the base ROS installation during Docker build:

1. **Repository Setup**
   ```bash
   # Add ROS 2 apt repository
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
   echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list
   ```

2. **Package Installation**
   - Uses `apt_packages` feature for optimized dependency management
   - Installs ROS 2 Jazzy desktop packages, colcon, rosdep, vcstool
   - Leverages BuildKit cache mounts for faster repeated builds

3. **Environment Configuration**
   - Sets `ROS_DISTRO=jazzy`
   - Configures rosdep and initializes ROS environment
   - No workspace directories created at system level

4. **Repository Discovery and Consolidation**
   - Scans the workspace for `*.repos` files during Docker build
   - Consolidates all found repository manifests into a single file
   ```bash
   # Scan for *.repos files in the build context
   find . -name "*.repos" -type f | while read repo_file; do
     echo "# From: $repo_file" >> consolidated.repos
     cat "$repo_file" >> consolidated.repos
   done
   ```
   - This happens at Docker image build time, not at container runtime

### Phase 2: User Environment Setup (`ros_jazzy_user_snippet.Dockerfile`)

The user snippet configures the per-user environment with all workspaces in the user's home directory:

1. **User Workspace Creation**
   ```bash
   # Consistent workspace structure for underlay and overlay
   ENV ROS_UNDERLAY_ROOT=$HOME/underlay
   ENV ROS_OVERLAY_ROOT=$HOME/overlay
   ENV ROS_WORKSPACE_ROOT=$HOME/overlay  # Main workspace for user development
   
   # Underlay workspace paths
   ENV ROS_UNDERLAY_PATH=$HOME/underlay/src
   ENV ROS_UNDERLAY_BUILD=$HOME/underlay/build
   ENV ROS_UNDERLAY_INSTALL=$HOME/underlay/install
   
   # Overlay workspace paths  
   ENV ROS_BUILD_BASE=$HOME/overlay/build
   ENV ROS_INSTALL_BASE=$HOME/overlay/install
   ENV ROS_LOG_BASE=$HOME/overlay/log
   
   # Create underlay workspace directories
   mkdir -p "$HOME/underlay/src" "$HOME/underlay/build" "$HOME/underlay/install" "$HOME/underlay/log"
   
   # Create overlay workspace directories
   mkdir -p "$HOME/overlay/src" "$HOME/overlay/build" "$HOME/overlay/install" "$HOME/overlay/log"
   
   # Collect user packages into overlay/src via symlinks
   find "$HOME" -maxdepth 1 -type d -name "*" -not -path "$HOME" -not -path "$HOME/underlay" -not -path "$HOME/overlay" | while read package_dir; do
     if [ -f "$package_dir/package.xml" ]; then
       ln -sf "$package_dir" "$HOME/overlay/src/$(basename "$package_dir")"
     fi
   done
   ```

2. **Unified Workspace Setup**
   - **Underlay**: Clones dependencies from `consolidated.repos` using vcstool into `$HOME/underlay/src`
   - **Overlay**: Collects user packages into `$HOME/overlay/src` via symlinks from mounted packages
   - **Unified scripts**: Uses `workspace_deps.sh` and `workspace_build.sh` for both underlay and overlay
   - **Consistent structure**: Both workspaces follow same `src/`, `build/`, `install/`, `log/` layout

3. **Unified Dependency Management**
   - **Both workspaces** use same `workspace_deps.sh` script for rosdep installation
   - **Runtime package collection**: Automatically symlinks mounted packages into `$HOME/overlay/src`
   - **Dynamic dependency resolution**: Detects new packages and installs dependencies as needed
   - **Script consistency**: Same dependency management logic for underlay and overlay

4. **Shell Integration**
   ```bash
   # Add to .bashrc for automatic environment sourcing
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   echo "[ -f \$HOME/underlay/install/setup.bash ] && source \$HOME/underlay/install/setup.bash" >> ~/.bashrc
   echo "[ -f \$HOME/overlay/install/setup.bash ] && source \$HOME/overlay/install/setup.bash" >> ~/.bashrc
   ```

### Phase 3: Runtime Operations

When the container starts, several automatic processes occur:

1. **Environment Sourcing Chain**
   - Base ROS: `/opt/ros/jazzy/setup.bash`
   - Underlay: `$HOME/underlay/install/setup.bash` (if exists after build)
   - User workspace: `$HOME/overlay/install/setup.bash` (if exists after build)

2. **Dynamic Dependency Resolution**
   ```bash
   # Automatic dependency installation using unified workspace approach
   if [ -d "$HOME/overlay/src" ] && [ "$(find $HOME/overlay/src -name package.xml)" ]; then
     workspace_deps.sh $HOME/overlay
   fi
   ```

3. **Actual Code Building** (Post-Container Creation)
   ```bash
   # Build underlay workspace first (if not already built)
   workspace_build.sh $HOME/underlay
   
   # Collect user packages into overlay workspace
   collect_packages.sh $HOME/overlay
   
   # Install dependencies for overlay packages
   workspace_deps.sh $HOME/overlay
   
   # Build overlay workspace - same unified approach
   workspace_build.sh $HOME/overlay
   
   # Or use colcon directly from overlay workspace
   cd $HOME/overlay
   colcon build
   colcon test  # Run tests
   colcon test-result --verbose  # Check test results
   source install/setup.bash
   ```
   - **Important**: Code compilation happens after the Docker container is created and running
   - **Script consistency**: Helper scripts (`underlay_build.sh`, `underlay_deps.sh`, `install_rosdeps.sh`) work both during Docker build and inside container
   - **Regular colcon usage**: Standard `colcon build`, `colcon test`, etc. work normally
   - **Colcon defaults**: Extra arguments like `--symlink-install`, `--cmake-args`, etc. are handled by colcon default configuration

### BuildKit Cache Optimization

The extension uses multiple BuildKit cache mounts for performance:

- **APT Cache**: `/var/cache/apt` and `/var/lib/apt/lists`
- **VCS Cache**: `/root/.cache/vcs-repos` for repository clones
- **Colcon Cache**: `/root/.colcon` for build artifacts
- **ROS Cache**: `/root/.ros` for rosdep and other ROS tools

### Build Timing Architecture

**Docker Build Stage (Image Creation):**
- Workspace scanning for `*.repos` files
- Repository consolidation
- System dependency installation via rosdep
- Environment setup and configuration
- **No code compilation** - only preparation

**Container Runtime (After Container Creation):**
- Actual code building with colcon
- Underlay workspace compilation using unified `workspace_build.sh`
- User workspace dependency installation using unified `workspace_deps.sh`
- Main workspace compilation
- Environment sourcing and activation
- **Script reuse**: Same build and dependency scripts available for manual execution

### Key Build Decisions

1. **User-Centric Design**: All workspaces (underlay and main) located in user home directory
2. **Generic Repository Handling**: No hardcoded repos, everything discovered dynamically
3. **Layered Dependencies**: Base ROS → User underlay → User workspace
4. **Permission Model**: User-writable paths for all development artifacts
5. **Build Stage Separation**: Dependency installation (Docker build) vs. code compilation (runtime)
6. **Script Consistency**: Same helper scripts work during Docker build and inside container
7. **Cache Strategy**: Aggressive caching at multiple levels for rebuild performance

### Common Build Scenarios

**Scenario 1: Static Workspace**
- repos files included in build context and scanned during Docker build
- Underlay dependencies installed via `workspace_deps.sh $HOME/underlay` during Docker build
- Overlay dependencies installed via `workspace_deps.sh $HOME/overlay` during Docker build
- Both workspaces can be pre-built using unified `workspace_build.sh` during Docker build
- User packages automatically collected into overlay structure via symlinks
- All unified scripts available inside container for manual execution

**Scenario 2: Dynamic Loading (`renv`)**
- No repos in build context during Docker build
- Workspaces empty initially with unified structure at `$HOME/underlay/` and `$HOME/overlay/`
- Dependencies resolved using unified `workspace_deps.sh` script at runtime
- Both workspaces built using unified `workspace_build.sh` script when loaded

**Scenario 3: Multi-Extension Build**
- vcstool extension provides repository management
- ros_underlay extension handles complex dependency chains
- ros_jazzy provides the foundation ROS environment
- All workspaces remain in user home directory

## Features

### Complete ROS 2 Installation
- **ROS 2 Jazzy Jalopy** from official apt repositories
- **Essential development tools**: colcon, rosdep, vcstool, ament tools
- **Automatic rosdep initialization** and configuration
- **Proper locale and timezone support** via dependency on `locales` extension

### Generic Design
- **Zero hardcoded files**: Works with any ROS repository out of the box
- **Dynamic repository discovery**: Automatically finds and processes `*.repos` files
- **Universal compatibility**: Tested with ros2/demos, kinisi-robotics/ros_devcontainer, and more
- **No project-specific hacks**: Maintains true genericity for any ROS workspace

### Standardized Workspace Layout
- **User-centric design**: All workspaces located in user home directory
- **Consistent structure**: Both underlay and overlay use same `src/`, `build/`, `install/`, `log/` layout
- **Underlay workspace**: `$HOME/underlay/` for dependency packages from `*.repos` files
- **Overlay workspace**: `$HOME/overlay/` for user development packages (symlinked from `$HOME/package_name/`)
- **Unified scripts**: Same workspace management scripts work for both underlay and overlay
- **Environment variables**: Clear separation between underlay and overlay paths

### Automatic Dependency Management
- **Repository consolidation**: Scans for `*.repos` files and consolidates them automatically
- **Underlay workspace creation**: Clones dependencies into `$HOME/underlay/src`
- **Multi-layer rosdep resolution**: Installs dependencies for both underlay and overlay workspaces
- **BuildKit cache optimization**: Uses cache mounts for faster dependency installation
- **Smart build ordering**: Underlay first (`$HOME/underlay/`), then overlay (`$HOME/overlay/`) using unified scripts

### Auto-Detection
The extension automatically activates when it detects ROS-related files:
- `package.xml` files (ROS packages)
- `*.repos` files (vcstool repository manifests)
- ROS-specific configuration files

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

## Integration with Other Extensions

### vcstool Extension
```bash
# vcstool is automatically included - no need to specify --vcstool
deps_rocker --ros-jazzy ubuntu:24.04
```
- **Automatic inclusion**: vcstool is a dependency of ros_jazzy and is automatically enabled
- **Shared workspace**: Both use `$HOME/ros_ws/` layout
- **Repository management**: vcstool imports dependencies into underlay `src/`, user packages collected into overlay `src/` via symlinks
- **Dependency coordination**: ros_jazzy provides ROS environment for vcstool operations

### ros_underlay Extension
```bash
deps_rocker --ros-jazzy --ros-underlay ubuntu:24.04
```
- **Dependency chain**: ros_underlay depends on ros_jazzy (vcstool included automatically)
- **Build coordination**: Underlay built first, then user workspace
- **Environment sharing**: Common workspace layout and environment variables

## Usage Examples

### Basic ROS Development
```bash
# Start container with ROS Jazzy
deps_rocker --ros-jazzy ubuntu:24.04

# Inside container - packages are mounted and automatically collected into overlay workspace
# For example, if you ran: deps_rocker --ros-jazzy --cwd ~/my_project ubuntu:24.04
# Your project is available at ~/my_project/ and symlinked in ~/overlay/src/my_project/

cd ~/overlay
colcon build  # Builds all packages in overlay workspace
colcon test   # Run tests
source install/setup.bash
```

### Dynamic Workspace Loading
```bash
# Load ROS demos dynamically - works with any ROS repository
renv ros2/demos

# Or any other ROS repository
renv kinisi-robotics/ros_devcontainer

# Dependencies are automatically discovered and installed
# Workspace is ready for building
colcon build
```

### Multi-Repository Workspace
```bash
# vcstool is automatically included with ros_jazzy
deps_rocker --ros-jazzy ubuntu:24.04

# Inside container - packages mounted via cwd extension and collected into overlay
# For example: deps_rocker --ros-jazzy --cwd ~/my_workspace ubuntu:24.04

# Use unified workspace scripts
workspace_deps.sh ~/overlay    # Install dependencies for overlay packages
workspace_build.sh ~/overlay   # Build overlay workspace

# Or use colcon directly
cd ~/overlay
colcon build  # Regular colcon commands with unified workspace structure
```

### Underlay + Overlay Workflow
```bash
# Container with underlay support (vcstool automatically included)
deps_rocker --ros-jazzy --ros-underlay ubuntu:24.04

# Underlay is built automatically during container creation in ~/underlay/install
# Overlay workspace builds on top of underlay using consistent structure
# Your packages are collected into ~/overlay/src/ via symlinks
cd ~/overlay
colcon build  # Uses underlay as base, builds overlay packages
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

### `/usr/local/bin/workspace_deps.sh`
Installs system dependencies for any workspace using rosdep.
```bash
workspace_deps.sh <workspace_path>
# Examples:
workspace_deps.sh $HOME/underlay   # Install deps for underlay packages
workspace_deps.sh $HOME/overlay    # Install deps for overlay packages
```
- **Unified approach**: Same script works for underlay, overlay, or any workspace
- **Consistency**: Identical behavior during Docker build and container runtime

### `/usr/local/bin/workspace_build.sh`
Builds any workspace from source packages with proper dependency resolution.
```bash
workspace_build.sh <workspace_path>
# Examples:
workspace_build.sh $HOME/underlay  # Build underlay workspace
workspace_build.sh $HOME/overlay   # Build overlay workspace
```
- **Generic functionality**: Works with any workspace structure
- **Flexible usage**: Can be called during Docker build or inside container

### `/usr/local/bin/collect_packages.sh`
Collects user packages into overlay workspace via symlinks.
```bash
collect_packages.sh <overlay_workspace_path>
# Example:
collect_packages.sh $HOME/overlay  # Symlink packages from $HOME into overlay/src/
```
- **Package discovery**: Finds ROS packages in `$HOME` and symlinks them into overlay
- **Dynamic updates**: Can be run anytime to collect newly mounted packages

### Legacy Scripts (for compatibility)
- `underlay_deps.sh` → calls `workspace_deps.sh $HOME/underlay`
- `underlay_build.sh` → calls `workspace_build.sh $HOME/underlay`
- `install_rosdeps.sh` → calls `workspace_deps.sh $HOME/overlay`

## Troubleshooting

### Permission Errors
**Problem**: Cannot create or write to workspace directories
**Solution**: All workspaces (including underlay) are created in user-writable `$HOME/ros_ws` directory with proper permissions

### Missing Dependencies
**Problem**: Colcon build fails with missing package errors
**Solution**: Run `rosdep install --from-paths src --ignore-src -r -y` to install system dependencies

### Environment Not Sourced
**Problem**: ROS commands not found
**Solution**: Source the setup file: `source /opt/ros/jazzy/setup.bash` or `source install/setup.bash`

### Dynamic Workspace Issues
**Problem**: `renv` loaded workspace has missing dependencies
**Solution**: The extension automatically detects and installs dependencies for dynamically loaded workspaces

## Dependencies

This extension automatically includes:
- **locales**: For proper ROS message handling and internationalization
- **vcstool**: For repository management (automatically included as a dependency)

## File Structure

```
deps_rocker/extensions/ros_jazzy/
├── __init__.py                     # Extension exports
├── ros_jazzy.py                    # Main extension class
├── ros_jazzy_snippet.Dockerfile    # System-level ROS installation
├── ros_jazzy_user_snippet.Dockerfile # User environment setup
├── auto_detect.yaml                # Auto-detection rules
├── install_rosdeps.sh              # Dependency installation script
├── underlay_build.sh               # Underlay build script
├── underlay_deps.sh                # Underlay dependency script
├── test.sh                         # Extension test script
├── spec.md                         # Technical specification
├── README.md                       # This file
└── configs/                        # Configuration files
```

## Testing

The extension includes comprehensive tests including generic compatibility:
```bash
# Run specific ros_jazzy tests
pytest test/test_extensions_generic.py::TestExtensions::test_ros_jazzy_extension -v

# Test auto-detection
pytest test/test_auto_detect_comprehensive.py -k ros_jazzy -v

# Test integration
./deps_rocker/extensions/ros_jazzy/test.sh

# Test generic compatibility with different repositories
renv ros2/demos -f -- colcon build
renv kinisi-robotics/ros_devcontainer -f -- colcon build
```

## Contributing

When modifying this extension:
1. Update the specification in `spec.md`
2. Test changes with `pixi run ci`
3. Ensure integration tests pass
4. Update documentation as needed

For more details, see the [Extension Implementation Checklist](../../AGENTS.md) in the project root.