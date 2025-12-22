# Spec: Optimize Pixi Build Performance

## Problem
The pixi extension has two slow build steps:
1. `RUN chmod -R a+rx /opt/pixi` - walks tens of thousands of files
2. Copying entire `.pixi/` directory including unnecessary caches and metadata

## Root Cause
1. **Slow chmod**: Docker's overlay filesystem makes recursive metadata operations extremely slow
2. **Over-copying**: Copying `.pixi/` includes task-cache and potentially multiple environments when only one is needed

The builder already uses cache mounts for package downloads (`/root/.cache/pixi` and `/root/.cache/rattler`), but we still copy runtime-unnecessary files.

## Solution
Two optimizations:

### 1. Use COPY --chmod (eliminates chmod -R)
Use `COPY --chmod` flag to set permissions during copy, eliminating separate chmod step.

### 2. Copy only specific environment (reduces file count)
Copy only `.pixi/envs/{env_name}/` instead of entire `.pixi/` directory, avoiding:
- `task-cache-v0/` - build-time cache, not needed at runtime
- Other environments if multiple exist in builder
- Metadata files

## Requirements
- Remove `RUN chmod -R a+rx @(pixi_runtime_root)` from pixi_snippet.Dockerfile
- Add `--chmod=755` to COPY commands for directories/binaries
- Add `--chmod=644` to COPY commands for config files
- Change `.pixi` copy to copy only `.pixi/envs/{env_name}`
- Maintain rocker compatibility (any UID must read/execute)
- Verify permissions work for non-root users

## Expected Outcome
Significantly faster builds by:
1. Eliminating recursive chmod walk
2. Reducing files copied by ~30-50% (no task-cache, metadata)
