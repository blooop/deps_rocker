# UV Extension: BuildKit Cache Mounts

The `uv` extension must leverage BuildKit cache mounts to enable a shared cache for improved efficiency during installation and dependency resolution. This ensures that repeated builds reuse cached downloads and artifacts, minimizing redundant network operations and speeding up container builds.

- Use `RUN --mount=type=cache,target=/root/.cache/uv` in the Dockerfile snippet for uv installation.
- The cache should persist between builds and be versioned if possible.
- The extension must remain compatible with the existing deps_rocker extension system.

# Plan: UV Extension BuildKit Cache Integration

## Goal
Enable the `uv` extension to use BuildKit cache mounts for a shared cache, improving build efficiency and reducing redundant downloads.

## Steps
1. **Update Dockerfile Snippet**
   - Add `RUN --mount=type=cache,target=/root/.cache/uv` to the uv installation step in the Dockerfile snippet.
   - Ensure the cache is versioned if possible (e.g., by uv version).
2. **Update Extension Logic**
   - Confirm the extension class and logic remain compatible with the new Dockerfile snippet.
3. **Testing**
   - Update or verify the test.sh script to check that uv works and the cache directory is used.
   - Run tests to ensure the extension installs and functions as expected.
4. **CI and Commit**
   - Run `pixi run ci` and fix any errors.
   - Commit and push changes using `pixi r fix-commit-push`.

## Acceptance Criteria
- The Dockerfile for uv uses a BuildKit cache mount for `/root/.cache/uv`.
- The extension passes all tests and CI checks.
- The cache is reused between builds, as verified by test.sh or build logs.
