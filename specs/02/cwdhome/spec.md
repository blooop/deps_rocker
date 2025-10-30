# CWD Home Extension Specification

## Purpose
Create a `cwdhome` extension that mounts the current working directory (CWD) into the container's home directory.

## Requirements
- Extension name: `cwdhome`
- Mount the host's current working directory to the container's home directory
- Allow users to work in their home directory with their local files
- No additional software installation required (pure volume mount)

## Implementation
- Inherit from `SimpleRockerExtension`
- Override volume mounting method to add CWD -> home mapping
- Set the working directory to the home directory
- No Dockerfile snippet needed (no installation required)
