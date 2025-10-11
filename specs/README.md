# Specifications

This document provides an index of all specifications for `deps_rocker`, grouped by topic.

## Core Enhancements

- [Multi-stage BuildKit support](./multi-stage-builds/README.md)
- [BuildKit Cache Mount Support](./buildkit-cache-mount/README.md)
- [Pip BuildKit Cache Mounts](./pip-buildkit-cache/README.md)

## Extensions

- [auto](./auto/README.md): Automatically detects and enables other extensions.
- [auto-search-root](./auto-search-root/README.md): Fixes the search root for the auto extension.
- [ccache-shared](./ccache-shared/README.md): Enables ccache compilation caching.
- [deps-dev](./deps-dev/README.md): Installs developer search tools.
- [jquery](./jquery/README.md): Installs `jq`.
- [neovim-version-pin](./neovim-version-pin/README.md): Pins the neovim version to fix a download issue.
- [nvim](./nvim/README.md): Installs neovim.
- [palanteer-dockerfile-copy-fix](./palanteer-dockerfile-copy-fix/README.md): Fixes a `COPY` command in the palanteer Dockerfile.
- [pixi-cache](./pixi-cache/README.md): Shares Pixi/uv cache from host to container.
- [uv](./uv/README.md): An extension for `uv` to use BuildKit cache.
- [vcstool-repo-cache](./vcstool-repo-cache/README.md): Caches vcstool repository clones.

## ROS (Robot Operating System)

- [ros-jazzy-dep-fix](./ros-jazzy-dep-fix/README.md): Fixes dependencies for the ROS Jazzy extension.
- [ros-underlay-builder](./ros-underlay-builder/README.md): Builds a shared ROS underlay.
- [ros-workspace-layout](./ros-workspace-layout/README.md): Defines the ROS workspace layout.