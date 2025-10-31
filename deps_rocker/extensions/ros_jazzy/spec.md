# ROS Jazzy Extension Specification

## Overview
The `ros_jazzy` extension provides a complete ROS 2 Jazzy Jalopy development environment with proper workspace layout, dependency management, and underlay build support. It installs ROS 2 Jazzy, sets up the workspace structure, and provides tools for building and managing ROS dependencies.

## Core Features

### 1. ROS 2 Jazzy Installation
- Installs ROS 2 Jazzy Jalopy from official apt repositories
- Uses `apt_packages` feature for proper dependency management and caching
- Includes all essential ROS development tools (colcon, rosdep, vcstool, etc.)
- Configures rosdep for system dependency resolution

### 2. Generic Design Principles
- **Zero project-specific hardcoded files**: Works with any ROS repository without customization
- **Dynamic repository discovery**: Scans for `*.repos` files and consolidates them automatically
- **Universal compatibility**: Supports arbitrary ROS repositories (ros2/demos, kinisi-robotics/ros_devcontainer, etc.)
- **No hardcoded paths**: Avoids project-specific hacks and maintains true genericity

### 3. Workspace Layout
- Adopts `/ros_ws` as the canonical workspace root
- Provides standardized subdirectories: `repos/`, `src/`, `underlay/`, `build/`, `install/`, `log/`
- Publishes workspace paths via environment variables (`ROS_WORKSPACE_ROOT`, etc.)
- Ensures compatibility across ros_jazzy, vcstool, and ros_underlay extensions

### 4. User Environment Setup
- Configures user-writable workspace directories in `$HOME/ros_ws`
- Handles environment variable redirection from system paths to user paths
- Provides automatic dependency installation for dynamically loaded workspaces
- Sources ROS environment automatically in user shell

### 5. Dependency Management
- **Dynamic repository scanning**: Automatically discovers and consolidates `*.repos` files in workspace
- **vcstool integration**: Clones dependencies into underlay workspace (`/opt/ros/underlay`)
- **Automatic rosdep resolution**: Installs system dependencies for both underlay and main workspace
- **Multi-layer workspace support**: Builds underlay dependencies first, then main workspace
- **BuildKit cache optimization**: Uses cache mounts for faster builds and dependency installation

## Dependencies
- `locales`: Required for proper ROS message handling
- `vcstool`: Required for repository management and workspace setup

## Environment Variables

### System-Level (Docker Build)
- `ROS_DISTRO=jazzy`
- `ROS_WORKSPACE_ROOT=/ros_ws`
- `ROS_UNDERLAY_PATH=/ros_ws/underlay`
- `ROS_UNDERLAY_BUILD=/ros_ws/underlay_build`
- `ROS_UNDERLAY_INSTALL=/ros_ws/underlay_install`
- `ROS_BUILD_BASE=/ros_ws/build`
- `ROS_INSTALL_BASE=/ros_ws/install`
- `ROS_LOG_BASE=/ros_ws/log`

### User-Level (Runtime)
- `ROS_WORKSPACE_ROOT=$HOME/ros_ws`
- All path variables redirected to user home directory
- `COLCON_LOG_PATH` and other build paths use writable workspace locations

## Generic Workflow

The extension follows a completely generic approach that works with any ROS repository:

1. **Repository Discovery**: Recursively scans workspace for `*.repos` files
2. **Dependency Consolidation**: Merges all found repos into `consolidated.repos`
3. **Underlay Creation**: Clones dependencies using vcstool into `/opt/ros/underlay`
4. **Dependency Installation**: Uses rosdep to install system dependencies for underlay
5. **Underlay Build**: Builds underlay workspace with colcon
6. **Main Workspace Prep**: Installs rosdep dependencies for main workspace packages
7. **Environment Setup**: Configures proper ROS environment sourcing chain

## Key Components

### 1. Main Dockerfile (`ros_jazzy_snippet.Dockerfile`)
- Installs ROS 2 Jazzy via apt packages
- Sets up system-level workspace directories and environment
- Configures rosdep and initializes ROS environment
- Provides base ROS installation for container

### 2. User Dockerfile (`ros_jazzy_user_snippet.Dockerfile`)
- Creates user-writable workspace directories
- Redirects environment variables to user home
- Provides automatic dependency installation for current workspace
- Handles dynamic workspace scenarios (e.g., `renv ros2/demos`)

### 3. Helper Scripts
- `install_rosdeps.sh`: Installs system dependencies for main workspace via rosdep
- `underlay_build.sh`: Builds underlay workspace with colcon
- `underlay_deps.sh`: Clones repos with vcstool and installs underlay dependencies
- `consolidated.repos`: Dynamically generated from workspace repo discovery

### 4. Auto-Detection (`auto_detect.yaml`)
- Automatically enables when ROS package files are detected
- Looks for `package.xml`, `*.repos` files, and ROS-specific configurations

## Integration Points

### With vcstool Extension
- Shares workspace layout conventions
- vcstool imports repositories into `src/` directory
- Preserves manifest copies in `repos/` directory
- Coordinates dependency resolution for imported repositories

### With ros_underlay Extension
- Provides base ROS environment for underlay building
- ros_underlay depends on ros_jazzy for ROS installation
- Shared workspace layout and environment variables
- Coordinated dependency resolution and build ordering

## Problem Areas Addressed

### 1. Workspace Permission Issues
**Problem**: Permission errors when creating workspace directories
**Solution**: Use `$HOME` directly in mkdir commands instead of environment variables that may not be fully processed

### 2. Dynamic Workspace Dependencies
**Problem**: Colcon build failures with missing dependencies in dynamically loaded workspaces
**Solution**: Check current workspace for packages and install dependencies via rosdep, skip underlay build for main workspace

### 3. Dependency Resolution Order
**Problem**: Wrong underlay packages built, causing missing dependencies
**Solution**: Proper dependency chain: vcstool → ros_jazzy → ros_underlay → user workspace

### 4. Environment Variable Consistency
**Problem**: Inconsistent workspace paths across extensions
**Solution**: Standardized environment variables shared across all ROS-related extensions

## Testing Requirements
- Verify ROS 2 Jazzy installation and basic functionality
- Test workspace directory creation and permissions
- Validate environment variable setup and sourcing
- Test integration with vcstool and dynamic workspaces
- Verify rosdep dependency resolution works correctly
- Test colcon build functionality in various scenarios
- **Generic compatibility tests**:
  - `renv ros2/demos -f -- colcon build` must work
  - `renv kinisi-robotics/ros_devcontainer -f -- colcon build` must work
  - Any arbitrary ROS repository should work without modification

## Usage Scenarios

### 1. Static Workspace Development
Traditional ROS development with pre-defined package structure and dependencies.

### 2. Dynamic Workspace Loading
Runtime loading of ROS repositories (e.g., `renv ros2/demos`) with automatic dependency resolution.

### 3. Underlay + Overlay Workflow
Multi-layered workspace with shared underlay dependencies and project-specific overlay packages.

### 4. CI/CD Integration
Automated testing and building of ROS packages in containerized environments.