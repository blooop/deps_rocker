# Investigation of Disk Space Issues

## Current State
Docker disk usage shows significant accumulation:
- Images: 129.6GB total (101.4GB reclaimable - 78%)
- Build Cache: 112GB total (81.62GB reclaimable)
- 24 cache mount locations across extensions
- Multiple cache IDs: pixi-global-cache, apt-cache, cargo-rustup-cache, claude-install-cache, conda-installer-cache, etc.

## Root Causes

1. **BuildKit Cache Accumulation**
   - Each extension uses cache mounts (`--mount=type=cache`)
   - 16+ unique cache IDs across all extensions
   - Caches never cleaned up between test runs
   - "All extensions together" test loads ALL caches simultaneously

2. **Multiple Builder Stages**
   - Each extension with `builder_pixi_packages` creates a builder stage
   - Builder stages create intermediate layers
   - Pixi global cache is shared but can grow large

3. **Test Image Accumulation**
   - Tests create many temporary images
   - Not all images are cleaned up after tests
   - Base images are rebuilt frequently

## Immediate Fix
- Delete isaac_sim extension (complex, not widely used)
- Reduces one builder stage and associated caches

## Potential Future Optimizations
1. Add cache size limits to mount points
2. Clean up Docker build cache in CI after each major test
3. Prune dangling images more aggressively
4. Consider consolidating some cache IDs (e.g., all pixi caches)
5. Add disk space monitoring to CI

## Implementation
1. Remove isaac_sim extension
2. Run CI to verify disk space is manageable
3. Monitor for future issues
