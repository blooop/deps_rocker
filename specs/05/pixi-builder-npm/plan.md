# Implementation Plan: Pixi Builder Pattern for NPM

## Current State
- npm extension uses `pixi global install nodejs` in user snippet
- Creates symlinks to `/usr/local/bin`
- Adds to PATH via profile/bashrc
- Pixi cache remains in the image

## Proposed Changes

### 1. Create `npm_builder_snippet.Dockerfile`
- Use builder stage (similar to pixi_builder_snippet.Dockerfile)
- Run `pixi global install nodejs` in builder
- Use cache mount for pixi cache to speed up builds
- Copy entire nodejs environment from `~/.pixi/envs/nodejs` to builder output directory

### 2. Create `npm_snippet.Dockerfile`
- Copy nodejs environment from builder stage to `/opt/deps_rocker/nodejs`
- Set proper permissions (readable by all users)

### 3. Update `npm_user_snippet.Dockerfile`
- Remove `pixi global install nodejs` command
- Copy from `/opt/deps_rocker/nodejs` to `~/.pixi/envs/nodejs`
- Keep symlinks and PATH configuration
- Keep environment variable configuration

### 4. Update `npm.py`
- Add `builder_apt_packages` if needed
- Follow pixi.py pattern for any template args

## Benefits
- Pixi cache excluded from final image (smaller images)
- Faster builds via cache mounts in builder stage
- Reusable pattern for other tools (uv, ruff, etc.)
- Better layer caching

## Testing
- Run existing test.sh to verify node/npm/npx work
- Verify CI passes
- Check image size reduction (optional)
