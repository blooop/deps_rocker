# CWD Home Extension Specification

## Purpose
Create a `cwdhome` extension that mounts the current working directory (CWD) inside the container's home directory.

## Requirements
- Extension name: `cwdhome`
- Mount the host's current working directory inside the container's home directory at `~/project_name`
- Set working directory to `~/project_name` in the container
- Preserves the container's home directory structure while adding the project
- No additional software installation required (pure volume mount)

## Example
If host is in `/host/path/projectA`, the container will:
- Mount it to `~/projectA`
- Start in `~/projectA` as the working directory

## Implementation
- Inherit from `SimpleRockerExtension`
- Depend on `user` extension to get home directory path
- Mount CWD to `~/project_name` where project_name is the current directory name
- Set the working directory to `~/project_name`
- No Dockerfile snippet needed (no installation required)
