# Foxglove Extension Implementation Plan

## Background
Foxglove Studio is a modern robotics data visualization and analysis tool that supports various formats like ROS bags, MCAP files, and live data streams. Installing it in a Docker container enables teams to standardize their robotics development environment.

## Implementation Steps

### 1. Extension Structure
- Create `deps_rocker/extensions/foxglove/` directory
- Implement `Foxglove` class inheriting from `SimpleRockerExtension`
- Set extension name to "foxglove"
- Add descriptive docstring for CLI help

### 2. Dependencies
- No direct extension dependencies needed
- System packages will be handled in Dockerfile snippet

### 3. Docker Installation
- Update apt package lists
- Install required system dependencies:
  - libnotify4: Desktop notifications
  - xdg-utils: Desktop integration utilities
  - libappindicator3-1: System tray support
- Download Foxglove Studio .deb package (v2.34.0)
- Install via dpkg
- Clean up downloaded package and apt cache

### 4. Testing
- Verify foxglove-studio command is available
- Check basic functionality (version check)
- Ensure no missing dependencies

### 5. Integration
- Add entry point to pyproject.toml
- Add to test suite in test_extensions_generic.py

## Technical Details

### Installation Method
Using the official .deb package ensures:
- Proper desktop integration
- Automatic dependency resolution
- Standard system service integration
- Future apt update compatibility

### Version Strategy
Starting with v2.34.0 (current latest) but the Dockerfile can be easily updated for newer versions.

### Container Considerations
- GUI application requiring X11 forwarding or similar for display
- Requires proper graphics drivers for 3D visualization
- Desktop integration dependencies for file associations