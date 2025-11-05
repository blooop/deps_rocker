# BuildKit Cache Strategy for CI Isolation

## Problem

When running parallel CI jobs (e.g., py310 and py313), using the same cache IDs with `sharing=locked` causes cross-job blocking:
- Job 1 holds lock → Job 2 waits indefinitely
- If Job 1 hangs → Job 2 times out
- Results in non-deterministic failures

## Solution: Dynamic Cache IDs with Build Args

### Implementation

**Dockerfile (`ros_jazzy_snippet.Dockerfile`)**:
```dockerfile
ARG CACHE_ID=default

RUN --mount=type=cache,id=apt-cache-${CACHE_ID},sharing=locked \
    apt-get update && apt-get install ...
```

### Usage

**Local development** (default):
```bash
# Uses CACHE_ID=default
rocker --ros-jazzy ubuntu:noble
```

**CI with isolation** (pass build arg):
```bash
# py310 job
docker build --build-arg CACHE_ID=py310 ...

# py313 job
docker build --build-arg CACHE_ID=py313 ...
```

### Benefits

1. **No Cross-Job Blocking**
   - py310 uses `apt-cache-py310`
   - py313 uses `apt-cache-py313`
   - Jobs run independently

2. **Better Cache Efficiency than `sharing=private`**
   - Each environment's cache is **reused across builds**
   - py310 job 1 → py310 job 2 uses same cache ✓
   - Much better than `sharing=private` which creates new cache per build

3. **Backwards Compatible**
   - Default `CACHE_ID=default` for users not passing build arg
   - Existing workflows continue to work

### Comparison

| Approach | Cross-Job Blocking | Cache Reuse | Complexity |
|----------|-------------------|-------------|------------|
| `sharing=locked` (same ID) | ❌ Yes | ✅ Best | Low |
| `sharing=private` | ✅ No | ❌ None | Low |
| **Dynamic ID** (this approach) | ✅ No | ✅ Good | Medium |

### GitHub Actions Integration

To use in CI, pass the matrix variable:

```yaml
strategy:
  matrix:
    environment: [py310, py313]

steps:
  - name: Build
    env:
      DOCKER_BUILDKIT: 1
    run: |
      # Rocker would need to support --build-arg or similar
      # Alternative: Set environment variable that rocker reads
      export ROCKER_CACHE_ID=${{ matrix.environment }}
      rocker --ros-jazzy ubuntu:noble
```

### Future Enhancement

If rocker doesn't support build args yet, can add via:
1. Environment variable: `ROCKER_CACHE_ID`
2. CLI flag: `rocker --cache-id py310 --ros-jazzy ...`
3. Auto-detect from `GITHUB_ACTIONS` env vars

## Current Status

✅ **Dockerfile updated** with `ARG CACHE_ID=default` and `${CACHE_ID}` in cache mount IDs
⏳ **Rocker integration**: Needs mechanism to pass CACHE_ID from CLI/env to Docker build
✅ **Fallback**: Works with default value for non-CI use
