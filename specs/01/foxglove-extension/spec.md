# Foxglove Extension Specification

## Overview
Add a Foxglove Studio extension to deps_rocker that installs Foxglove Studio for robotics data visualization and analysis.

## Requirements
- Install Foxglove Studio (latest) on Linux using snap
- Use snap package installation method for reliability
- Mount persistent storage for Foxglove Agent
- Follow deps_rocker extension patterns using SimpleRockerExtension

## Implementation
- Extension name: `foxglove`
- Inherit from SimpleRockerExtension
- Install via snap package manager for easy maintenance
- Mount volume for agent index: `foxglove-agent-index:/index`
- Mount recordings directory: `${HOME}/foxglove_recordings:/storage`
- Include proper testing to verify snap installation