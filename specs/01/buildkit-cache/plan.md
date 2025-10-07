# Plan: BuildKit Extension Caching

1. **Analyze Current Extension Dockerfile Patterns**
   - Review how extensions currently generate Dockerfile snippets.
   - Identify steps suitable for caching (downloads, installs, clones).

2. **Design Cache Mount Strategy**
   - Specify cache mount locations and naming conventions for extensions.
   - Define versioned cache keys for each extension step.

3. **Update Extension Templates**
   - Add BuildKit cache mount examples to extension Dockerfile templates.
   - Document usage for extension authors.

4. **Implement Shared Cache Logic**
   - Ensure cache mounts are reused across builds with overlapping extensions.
   - Support both local and remote cache sources in build scripts.

5. **Test Cache Effectiveness**
   - Build containers with different extension sets and measure cache reuse.
   - Validate that common steps are deduplicated and cached.

6. **Document Best Practices**
   - Write guidelines for extension authors on cache usage and invalidation.
   - Add troubleshooting for cache misses and stale cache issues.

7. **Open Questions & Next Steps**
   - Address coordination of cache keys and invalidation strategies.
   - Plan for future remote cache support if needed.
