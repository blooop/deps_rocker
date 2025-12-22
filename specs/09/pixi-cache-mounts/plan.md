# Implementation Plan: Unconditional Pixi BuildKit Cache Mounts

## Current State
- Cache mounts are conditional: `if pixi_use_cache_mounts else ""`
- `pixi_use_cache_mounts` is set based on `is_buildkit_enabled()` during config
- BuildKit is ensured during build via `docker_cli.py`, but this happens after config

## Analysis
The timing issue:
1. Extension config happens first → checks `DOCKER_BUILDKIT` → might be False
2. Dockerfile template is generated → may not include cache mounts
3. Build happens later → `docker_cli.py` ensures `DOCKER_BUILDKIT=1`
4. Too late - Dockerfile already generated without cache mounts

## Changes Needed

### 1. Update pixi.py
- Remove `cache_mounts = is_buildkit_enabled()` line
- Always set `cache_mounts = True`
- Remove conditional logic that makes cache mounts optional

### 2. Update pixi_builder_snippet.Dockerfile
- Keep the cache mount syntax (already correct)
- Remove the `if pixi_use_cache_mounts` conditional from template
- Always emit cache mount directives

### 3. Documentation
- Update README to document BuildKit requirement
- Add note that `DOCKER_BUILDKIT=1` is recommended but handled automatically
- Explain cache behavior and cross-project reuse

## Cache Mount Configuration

Current (correct) cache mounts:
```dockerfile
RUN --mount=type=cache,target=/root/.cache/pixi,id=pixi-cache \
    --mount=type=cache,target=/root/.cache/rattler,id=rattler-cache \
    ...
```

These are already optimal:
- Stable IDs (not project-specific) ✓
- Correct target paths for Pixi/Rattler caches ✓
- Reusable across all projects ✓

No need to add solver-cache mount (not a standard Pixi cache directory per web research).

## Testing
- Verify builds work with BuildKit
- Verify builds fail gracefully without BuildKit (or add explicit check)
- Test cache reuse across different projects
- Run full CI suite
