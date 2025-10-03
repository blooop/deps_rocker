# Foxglove Extension Specification

## Overview
Add a Foxglove Studio extension to deps_rocker that installs Foxglove Studio for robotics data visualization and analysis.

## Requirements
- Install Foxglove Studio v2.15.0 (latest) on Linux amd64
- Handle required system dependencies (libnotify4, xdg-utils, libappindicator3-1)
- Use .deb package installation method
- Mount persistent storage for Foxglove Agent
- Follow deps_rocker extension patterns using SimpleRockerExtension

## Implementation
- Extension name: `foxglove`
- Inherit from SimpleRockerExtension
- Install via .deb package from official Foxglove releases
- Clean up package file after installation
- Mount volume for agent index: `foxglove-agent-index:/index`
- Mount recordings directory: `${HOME}/foxglove_recordings:/storage`
- Include proper testing to verify installation