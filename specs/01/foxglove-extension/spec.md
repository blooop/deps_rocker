# Foxglove Extension Specification

## Overview
Add a Foxglove Studio extension to deps_rocker that installs Foxglove Studio for robotics data visualization and analysis.

## Requirements
- Install Foxglove Studio v2.34.0 on Linux using direct .deb download
- Use reliable download with retry and validation
- Mount persistent storage for Foxglove Agent
- Follow deps_rocker extension patterns using SimpleRockerExtension

## Implementation
- Extension name: `foxglove`
- Inherit from SimpleRockerExtension
- Install via direct .deb download from official Foxglove releases
- Verify package integrity before installation
- Mount volume for agent index: `foxglove-agent-index:/index`
- Mount recordings directory: `${HOME}/foxglove_recordings:/storage`
- Include proper testing to verify package installation