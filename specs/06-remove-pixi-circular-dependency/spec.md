# Remove pixi circular dependency

Remove circular dependency where pixi is being used to install dependencies (curl, ca-certificates) needed to install pixi itself.

## Solution

Copy pixi binary from official pixi Docker image (`ghcr.io/prefix-dev/pixi:latest`) instead of installing via script. This:
- Eliminates need for curl/ca-certificates in builder stages
- Removes apt usage, significantly speeding up builds
- Works for both builder stages and pixi extension

## Changes

- Update `SimpleRockerExtension._get_builder_pixi_snippet()` to use multi-stage FROM to copy pixi binary
- Remove `builder_pixi_packages = ["curl", "ca-certificates"]` from pixi extension
- Ensure pixi binary is executable and in PATH
