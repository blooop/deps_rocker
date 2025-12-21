# Pixi build-time tools env (workflow)

- Make pixi the primary package backend by building a single `tools` environment during Docker image build via a pixi-docker builder stage, copying the realized env into the runtime image, and wiring PATH so tools work without activation for root and the rocker user.
- Preserve dynamic extension detection/merging: extensions declare pixi deps that are deduped into one generated `pixi.toml`/`pixi.lock` aware install (`--locked` when available); keep non-pixi backends only as fallbacks for OS-level needs.
- Keep rocker runtime behavior intact (GPU/drivers, audio, user mapping, X11/devices, entrypoints) and remove reliance on runtime pixi sync/global installs or entrypoint hacks.
- Provide configuration for selecting the pixi builder image/tag; prefer a compatible default (e.g., pixi jammy) and ensure Dockerfile generation adds the builder stage, install, copy, PATH, and caching-friendly layering.
- Update docs/tests to reflect the build-time pixi flow, PATH behavior, lockfile handling, and extension contribution model; validate Dockerfile output and extension-driven dependency merging.
