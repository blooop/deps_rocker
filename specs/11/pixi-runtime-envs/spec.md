# Spec 11: Pixi Runtime Environments with Persistent Volumes

## Problem
Current approach of baking pixi environments into Docker image layers is slow and defeats pixi's caching advantages:
- Writing 50k+ conda package files into Docker layers is slow
- Hardlinks can't span Docker layers, forcing duplication
- Every dependency change requires rebuilding entire images
- Pixi's cross-project cache reuse is lost

## Root Cause
- Docker image layers break pixi's hardlink/reflink optimization
- Baking environments at build time prevents persistent cache reuse
- Image rebuilds for every dependency change waste time
- No sharing of common packages across different projects

## Requirements
- Maintain full Docker isolation and rocker features (GPU, audio, user mapping)
- Achieve pixi-speed dependency installs via persistent caching
- Enable dynamic dependency detection (renv-style)
- Fast container reuse when dependencies haven't changed
- Support both dev (fast iteration) and CI (reproducible) workflows

## Solution
Shift pixi from build-time to runtime:

1. **Thin base image**: Contains only rocker features + pixi binary, no baked environments
2. **Persistent volumes**: Docker-managed volumes for pixi cache and detached environments
3. **Runtime install**: Run `pixi install` at container startup using persistent cache
4. **Container fingerprinting**: Reuse existing containers when dependency hash matches
5. **Feature-based manifests**: Map detected extensions to pixi features for dynamic composition

## Architecture
```
Thin Docker Image (rocker base + pixi binary)
    ↓
Docker Volume: /opt/pixi (cache + detached envs on same filesystem)
    ↓
Container Startup: pixi install --locked (fast with warm cache)
    ↓
Environment Activation: eval $(pixi shell-hook)
```

## Success Criteria
- First install of new dependencies: moderate speed (downloads packages)
- Subsequent installs with warm cache: very fast (just hardlinks)
- Changing project dependencies: no image rebuild, just `pixi install`
- Multiple projects share common packages via persistent cache
- Container reuse when dependencies unchanged: instant attach
- CI mode: reproducible with `--frozen` flag
