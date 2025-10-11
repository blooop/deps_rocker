# Docker-in-Docker with Full Isolation

## Goal
Provide a fully isolated Docker environment inside containers that requires only extension activation, with complete separation from the host Docker daemon.

## Requirements
- **One-step activation**: Only `--docker-in-docker` flag needed, no additional caveats
- **Complete isolation**: Separate Docker daemon inside container, independent from host
- **Automatic configuration**: Extension automatically provides all required Docker arguments
- **No host contamination**: Container's Docker operations must not affect host Docker
- **Transparent operation**: Docker commands work seamlessly inside container

## Architecture Decision: True DinD vs DooD

### Rejected Approach: Docker-out-of-Docker (DooD)
- mp_rocker uses DooD: mounts host's `/var/run/docker.sock`
- **Problem**: Shares host Docker daemon - NO ISOLATION
- **Risk**: Container's Docker operations affect host system
- **Example**: Container can see and control host's containers

### Selected Approach: True Docker-in-Docker (DinD)
- Runs separate `dockerd` inside container
- **Benefit**: Complete isolation from host
- **Requirement**: `--privileged` mode (auto-provided by extension)
- **Storage**: Separate `/var/lib/docker` volume for container's Docker data

## Technical Requirements
- Install Docker CE, CLI, Compose plugin, and Buildx plugin
- Automatically add `--privileged` flag
- Automatically create volume for `/var/lib/docker`
- Provide entrypoint script to start `dockerd` automatically
- Handle non-root users via docker group membership
- Support graceful daemon shutdown

## Success Criteria
- User runs: `rocker --docker-in-docker ubuntu:22.04`
- Container starts with Docker daemon running
- Docker commands work immediately: `docker ps`, `docker run hello-world`
- Container's Docker operations are invisible to host
- CI tests pass with privileged mode
