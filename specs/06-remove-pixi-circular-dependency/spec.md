# Remove pixi circular dependency

Remove circular dependency where pixi is being used to install dependencies (curl, ca-certificates) needed to install pixi itself.

## Solution

Copy pixi binary from official pixi Docker image (`ghcr.io/prefix-dev/pixi:latest`) instead of installing via script. This:
- Eliminates need for curl/ca-certificates in builder stages
- Removes apt usage, significantly speeding up builds
- Works for both builder stages and pixi extension

## Changes Made

1. Updated `SimpleRockerExtension._get_builder_pixi_snippet()` to copy pixi from official Docker image
2. Removed `curl` and `ca-certificates` from `builder_pixi_packages` in all extensions (pixi, cargo, claude, nvim, npm, conda, palanteer, urdf_viz)
3. Updated pixi builder snippet to copy pixi binary to `.pixi/bin` directory structure
4. Enabled BuildKit in test suites to support cache mounts
5. Updated pyproject.toml test tasks to enable BuildKit by default
