# Spec: Share Pixi/uv cache from host to container

## Goal
Ensure that the Pixi and uv caches are mounted from the host into the container to speed up downloads and builds.

## Requirements
- Pixi cache (typically ~/.cache/pixi) and uv cache (typically ~/.cache/uv) should be mounted as Docker cache volumes.
- The Dockerfile snippet for the pixi extension should use BuildKit cache mounts for these directories.
- The cache should persist between builds and be reused for subsequent runs.
- The implementation should work for both host and CI environments.
- Update documentation if needed.

## Out of Scope
- Changing Pixi/uv configuration beyond cache mounting.
- Supporting non-standard cache locations.

## References
- See AGENTS.md for BuildKit cache mount patterns.
- See existing Dockerfile snippets for similar cache usage.
