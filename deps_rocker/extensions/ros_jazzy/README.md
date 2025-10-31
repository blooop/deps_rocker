# ROS Jazzy Extension

The `ros_jazzy` extension provides a complete ROS 2 Jazzy Jalopy development environment for Docker containers, with proper workspace layout, dependency management, and integration with other deps_rocker extensions.

## Quick Start

```bash
# Basic ROS Jazzy environment
deps_rocker --ros-jazzy ubuntu:22.04

# With vcstool for repository management
deps_rocker --ros-jazzy --vcstool ubuntu:22.04

# With additional development tools
deps_rocker --ros-jazzy --vcstool --git-clone --nvim ubuntu:22.04
```

## Features

### 🚀 Complete ROS 2 Installation
- **ROS 2 Jazzy Jalopy** from official apt repositories
- **Essential development tools**: colcon, rosdep, vcstool, ament tools
- **Automatic rosdep initialization** and configuration
- **Proper locale and timezone support** via dependency on `locales` extension

### 🎯 Generic Design
- **Zero hardcoded files**: Works with any ROS repository out of the box
- **Dynamic repository discovery**: Automatically finds and processes `*.repos` files
- **Universal compatibility**: Tested with ros2/demos, kinisi-robotics/ros_devcontainer, and more
- **No project-specific hacks**: Maintains true genericity for any ROS workspace

### 📁 Standardized Workspace Layout
- **System workspace**: `/ros_ws/` with subdirectories for repos, src, underlay, build, install, log
- **User workspace**: `$HOME/ros_ws/` for user-writable development
- **Environment variables**: `ROS_WORKSPACE_ROOT`, `ROS_BUILD_BASE`, `ROS_INSTALL_BASE`, etc.
- **Consistent layout** across all ROS-related extensions

### 🔧 Automatic Dependency Management
- **Repository consolidation**: Scans for `*.repos` files and consolidates them automatically
- **Underlay workspace creation**: Clones dependencies into `/opt/ros/underlay`
- **Multi-layer rosdep resolution**: Installs dependencies for both underlay and main workspace
- **BuildKit cache optimization**: Uses cache mounts for faster dependency installation
- **Smart build ordering**: Underlay first, then main workspace

### 🎯 Auto-Detection
The extension automatically activates when it detects ROS-related files:
- `package.xml` files (ROS packages)
- `*.repos` files (vcstool repository manifests)
- ROS-specific configuration files

## Workspace Structure

### System Level (Docker Build)
```
/ros_ws/
├── repos/          # vcstool manifests and metadata
├── src/            # Source packages (from vcstool imports)
├── underlay/       # Underlay source packages
├── underlay_build/ # Underlay build artifacts
├── underlay_install/ # Underlay install space
├── build/          # Main workspace build artifacts
├── install/        # Main workspace install space
└── log/            # Build and test logs
```

### User Level (Runtime)
```
$HOME/ros_ws/
├── repos/          # User-writable manifests
├── src/            # User development packages
├── build/          # User build artifacts
├── install/        # User install space
└── log/            # User build logs
```

## Integration with Other Extensions

### vcstool Extension
```bash
deps_rocker --ros-jazzy --vcstool ubuntu:22.04
```
- **Shared workspace**: Both use `/ros_ws/` layout
- **Repository management**: vcstool imports into `src/`, preserves manifests in `repos/`
- **Dependency coordination**: ros_jazzy provides ROS environment for vcstool operations

### ros_underlay Extension
```bash
deps_rocker --ros-jazzy --vcstool --ros-underlay ubuntu:22.04
```
- **Dependency chain**: ros_underlay depends on ros_jazzy
- **Build coordination**: Underlay built first, then user workspace
- **Environment sharing**: Common workspace layout and environment variables

## Usage Examples

### Basic ROS Development
```bash
# Start container with ROS Jazzy
deps_rocker --ros-jazzy ubuntu:22.04

# Inside container
cd /ros_ws/src
git clone https://github.com/ros2/examples.git
cd /ros_ws
colcon build
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
# Create workspace with vcstool
deps_rocker --ros-jazzy --vcstool ubuntu:22.04

# Inside container - import repositories
cd /ros_ws
vcs import src < repos/my_workspace.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Underlay + Overlay Workflow
```bash
# Container with underlay support
deps_rocker --ros-jazzy --vcstool --ros-underlay ubuntu:22.04

# Underlay is built automatically during container creation
# User workspace builds on top of underlay
cd /ros_ws/src
# Add your packages here
cd /ros_ws
colcon build  # Uses underlay as base
```

## Environment Variables

### Workspace Paths
- `ROS_WORKSPACE_ROOT`: Main workspace directory (`/ros_ws` or `$HOME/ros_ws`)
- `ROS_BUILD_BASE`: Build artifacts directory
- `ROS_INSTALL_BASE`: Install space directory
- `ROS_LOG_BASE`: Log files directory

### Underlay Paths
- `ROS_UNDERLAY_PATH`: Underlay source directory
- `ROS_UNDERLAY_BUILD`: Underlay build directory
- `ROS_UNDERLAY_INSTALL`: Underlay install directory

### ROS Configuration
- `ROS_DISTRO=jazzy`: ROS distribution identifier
- `COLCON_LOG_PATH`: Colcon log directory (uses workspace log path)

## Helper Scripts

### `/usr/local/bin/install_rosdeps.sh`
Installs system dependencies for ROS packages using rosdep.
```bash
install_rosdeps.sh /path/to/workspace/src
```

### `/usr/local/bin/underlay_build.sh`
Builds underlay workspace with proper dependency resolution.
```bash
underlay_build.sh
```

### `/usr/local/bin/underlay_deps.sh`
Clones repositories with vcstool and installs underlay dependencies via rosdep.
```bash
underlay_deps.sh
```

## Troubleshooting

### Permission Errors
**Problem**: Cannot create directories in `/ros_ws`
**Solution**: The extension automatically creates user-writable workspace in `$HOME/ros_ws`

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

This extension requires:
- **locales**: For proper ROS message handling and internationalization
- **vcstool**: For repository management (automatically pulled in via dependency)

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