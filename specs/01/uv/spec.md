# UV Extension: BuildKit Cache Mounts

The `uv` extension must leverage BuildKit cache mounts to enable a shared cache for improved efficiency during installation and dependency resolution. This ensures that repeated builds reuse cached downloads and artifacts, minimizing redundant network operations and speeding up container builds.

- Use `RUN --mount=type=cache,target=/root/.cache/uv` in the Dockerfile snippet for uv installation.
- The cache should persist between builds and be versioned if possible.
- The extension must remain compatible with the existing deps_rocker extension system.
