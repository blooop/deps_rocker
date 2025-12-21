# Use Official Pixi Docker Image

## Goal
Update pixi and npm extensions to use the official pixi Docker image instead of downloading/installing pixi from scratch.

## Changes

### Pixi Extension
- Replace curl-based installation with copying binaries from `ghcr.io/prefix-dev/pixi:latest`
- Use multi-stage build pattern to extract pixi from official image
- Maintain same functionality (shell completion, PATH setup)

### NPM Extension
- Add builder stage that uses official pixi image
- Install nodejs via pixi in builder stage
- Copy nodejs/npm binaries to user stage
- Remove runtime dependency on pixi installation

## Benefits
- Faster builds (no download/install step)
- Use officially maintained pixi binaries
- Better caching for npm (separate builder stage)
- Reduced image size for npm-only users
