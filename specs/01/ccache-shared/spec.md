# Shared ccache Extension

## Goal
Enable ccache compilation caching shared between the host machine and all containers to speed up C/C++ builds.

## Requirements
- Install ccache in the Docker image
- Mount host ccache directory into container at `~/.ccache`
- Ensure ccache directory is created on host if it doesn't exist
- Set CCACHE_DIR environment variable to point to the mounted directory
- Allow cache to be shared across multiple container instances

## Implementation
- Create ccache extension that inherits from SimpleRockerExtension
- Override `get_snippet()` to install ccache
- Override `get_docker_args()` to add volume mount for ccache directory
- Default host ccache location: `~/.ccache`
- Container ccache location: `/home/{user}/.ccache`
