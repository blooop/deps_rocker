# ROS Repos Underlay Extension

## Overview
Automatically scan for `*.repos` files and set them up as composable ROS underlays, enabling seamless dependency resolution for ROS packages.

## Goals
- Scan working directory for `*.repos` files
- Clone repositories using vcstool if not already present
- Build repositories as ROS underlays in dependency order
- Source underlays in correct order for package discovery
- Support recursive dependency resolution (repos files in dependencies)

## Key Features
- **Auto-discovery**: Find all `*.repos` files in workspace
- **Composable**: Each dependency's repos file automatically processed
- **Ordering**: Build and source underlays in correct dependency order
- **Caching**: Reuse existing builds when possible
- **Integration**: Works with existing vcstool and ROS extensions

## Usage
```bash
rocker --ros-repos-underlay ubuntu:22.04
```

## Dependencies
- `vcstool` extension for repository cloning
- `ros_generic` or specific ROS distro extension for build tools
- `git` for repository operations

## Expected Behavior
1. Scan for `*.repos` files in workspace
2. For each repos file: clone missing repositories
3. Recursively process any `*.repos` files in cloned dependencies
4. Build all repositories as ROS workspaces in dependency order
5. Source built workspaces as underlays
6. Configure environment for main workspace to use all underlays
