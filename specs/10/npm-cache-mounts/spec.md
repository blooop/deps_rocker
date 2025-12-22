# Spec 10: NPM BuildKit Cache Mounts for Gemini and Codex

## Problem
Gemini and Codex extensions are slow to install because npm packages are re-downloaded on every build, even when the packages haven't changed.

## Root Cause
- `npm install -g` commands in Dockerfiles don't use BuildKit cache mounts
- NPM downloads package tarballs to `/root/.npm` cache but this cache is lost between builds
- Each build re-downloads the same packages from the npm registry

## Requirements
- Cache npm's package download cache across builds
- Speed up subsequent builds after cache is warmed
- Maintain identical final image (no cache in runtime layers)
- Follow BuildKit best practices

## Solution
- Add `--mount=type=cache,target=/root/.npm,id=npm-cache` to npm install RUN commands
- Use stable cache ID `npm-cache` shared across all npm-using extensions
- Cache directory is only used during build, not included in final layers

## Success Criteria
- First build downloads packages (acceptable)
- Second build reuses cached packages (faster)
- Different projects with same npm packages benefit from shared cache
- Final image size unchanged
