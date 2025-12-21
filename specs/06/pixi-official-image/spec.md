# Use Official Pixi Docker Image

## Goal
Update pixi and npm extensions to use the official pixi Docker image instead of downloading/installing pixi from scratch.

## Changes

### Pixi Extension
- Use `ghcr.io/prefix-dev/pixi:latest` as source for pixi binary
- Copy pixi directly to `/usr/local/bin/pixi` (globally accessible)
- Remove curl dependency (no longer needed)
- Remove PIXI_VERSION empy_args (use latest from official image)
- Simplified user stage (no `.pixi` directory copying)

### NPM Extension
- **Builder stage**: Use official pixi image to install nodejs via `pixi global install`
- **System stage (`npm_snippet.Dockerfile`)**:
  - Copy nodejs to `/opt/nodejs`
  - Run `chmod -R a+rX` to set permissions (as root, faster than sudo in user stage)
  - Create symlinks in `/usr/local/bin` for global access
  - Set `OPENSSL_CONF` env var for SSL config paths
- **User stage**:
  - Configure npm to use `~/.npm-global` prefix for user packages
  - Add `~/.npm-global/bin` to PATH
- Remove pixi runtime dependency (only used in builder)

### Codex and Gemini Extensions
- Fix symlinks to point to `~/.npm-global/bin/` instead of `~/.pixi/envs/nodejs/bin/`

## Benefits
- ✅ Faster builds (official pixi image is cached)
- ✅ No download/install step for pixi
- ✅ Tools available during build (symlinks in `/usr/local/bin`)
- ✅ Tools available at runtime without activation
- ✅ Works with renv/rockerc pattern
- ✅ Proper npm permissions (user-writable prefix)

## Trade-offs
- chmod step takes ~26s (but runs as root in system stage, not sudo in user stage)
- Test dependency chain changed (curl no longer transitive dependency)
