# Plan: Share Pixi/uv cache from host to container

1. Review current pixi extension Dockerfile snippet for cache usage.
2. Identify cache directories for pixi (~/.cache/pixi) and uv (~/.cache/uv).
3. Update pixi_snippet.Dockerfile to use BuildKit cache mounts for both directories.
4. Ensure cache mounts are used for all relevant install/download steps.
5. Test that cache is reused between builds (locally and in CI).
6. Update documentation/spec as needed.
7. Commit only the contents of specs/01/pixi-cache.
