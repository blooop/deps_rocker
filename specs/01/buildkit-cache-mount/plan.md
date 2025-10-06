# BuildKit Cache Mount Implementation Plan

## Current State
- `SimpleRockerExtension.get_apt_command()` has infrastructure for cache mounts
- Currently defaults to `use_cache_mount=False`
- Need to auto-detect BuildKit from environment

## Implementation Steps

### 1. Update `get_snippet()` method
- Check `DOCKER_BUILDKIT` environment variable
- Pass buildkit flag to `get_apt_command()`

### 2. Modify `get_apt_command()`
- When `use_cache_mount=True`: Generate RUN with cache mounts
- When `use_cache_mount=False`: Generate traditional RUN with cleanup

### 3. Testing Strategy
- Test with `DOCKER_BUILDKIT=1`: Should generate cache mount syntax
- Test without BuildKit: Should generate traditional syntax with cleanup
- Verify CI passes in both modes

## Technical Details

### Cache Mount Syntax (BuildKit enabled)
```dockerfile
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    <packages>
```

### Traditional Syntax (BuildKit disabled)
```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    <packages> \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
```

## Benefits
- Faster builds with BuildKit through persistent apt cache
- Backward compatible with Docker without BuildKit
- No changes needed to existing extensions
