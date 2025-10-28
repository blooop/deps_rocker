# BuildKit Cache Mount Support

## Goal
Enable Docker BuildKit cache mounting in SimpleRockerExtension to speed up apt package installations.

## Requirements
- When `DOCKER_BUILDKIT=1` environment variable is set, generate Dockerfile with cache mount syntax
- Use `--mount=type=cache,target=/var/cache/apt,sharing=locked` for apt cache
- Use `--mount=type=cache,target=/var/lib/apt/lists,sharing=locked` for apt lists
- Fall back to traditional apt install with cleanup when BuildKit is not enabled
- Maintain backward compatibility with existing extensions

## Implementation
- Detect `DOCKER_BUILDKIT=1` environment variable in `get_snippet()` method
- Pass buildkit flag to `get_apt_command()`
- Generate appropriate RUN command based on buildkit availability
