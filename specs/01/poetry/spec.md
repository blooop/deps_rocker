# Poetry Extension Spec

## Goal
Implement a Poetry extension for Python dependency management, inspired by uv and pixi extensions.

## Features
- Installs Poetry in Docker using best practices and cache mounts
- Provides a SimpleRockerExtension class for Poetry
- Registers as a rocker extension entry point
- Includes generic and custom tests

## Acceptance Criteria
- Poetry is installed and available in the container
- Extension is discoverable and testable via rocker
- Follows project extension conventions
