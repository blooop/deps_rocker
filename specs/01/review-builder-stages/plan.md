# Plan: Optimize Builder Stage Cache Efficiency

## Recommendations

### Priority 1: Fixed Base Image for Builder Stages

**Goal**: Decouple builder stages from final image base to prevent unnecessary cache invalidation.

**Implementation**:
1. Add class attribute `builder_base_image` to SimpleRockerExtension (default: `ubuntu:22.04`)
2. Update `_build_template_args()` to provide `builder_base_image` to templates
3. Modify builder snippets to use: `FROM {builder_base_image} AS {builder_stage}`
4. Document when extensions need specific builder base (e.g., glibc version compatibility)

**Benefits**:
- Changing final base image (22.04 → 24.04) won't invalidate builder caches
- Cross-project cache sharing when using same builder base
- Palanteer and similar tools can truly cache across different final images

**Compatibility**:
- Backward compatible (defaults to current behavior if not specified)
- Extensions can override for specific requirements

**Example**:
```python
class Cargo(SimpleRockerExtension):
    name = "cargo"
    builder_base_image = "ubuntu:22.04"  # Fixed for cache stability
    # Cargo binaries work across Ubuntu versions with compatible glibc
```

### Priority 2: Stable Builder Stage Ordering

**Goal**: Prevent extension order changes from breaking unrelated builder stage caches.

**Problem**: Docker caches multi-stage builds by line number. Reordering breaks cache.

**Options**:

#### Option A: Alphabetical Builder Emission (Simple)
- Emit builder stages in alphabetical order by extension name
- Pros: Deterministic, easy to implement, no config needed
- Cons: Not optimal for cache (frequently-changing extensions might be early)

#### Option B: Explicit Cache Priority (Optimal)
- Add `cache_priority` attribute (0-100, default 50)
- Sort builders by: priority (desc) → alphabetical
- Extensions that change rarely (npm, cargo) get high priority (early emission)
- Extensions that change often (project-specific) get low priority (late emission)

**Recommended**: Option B with sensible defaults.

**Implementation**:
```python
class SimpleRockerExtension:
    cache_priority = 50  # Default medium priority

class Cargo(SimpleRockerExtension):
    cache_priority = 90  # Rarely changes, emit early

class ProjectDeps(SimpleRockerExtension):
    cache_priority = 10  # Changes often, emit late
```

**Benefits**:
- Stable builders emit first → cache persists across extension list changes
- New extensions don't invalidate existing builder caches
- Users can tune priority for their workflow

### Priority 3: Group Final Stage Operations

**Goal**: Minimize cache breaks in final stage when extension order changes.

**Strategy**: Reorder final stage operations to group similar operations:
1. All apt-get install commands (parallelizable, quick)
2. All COPY --from=builder commands (no dependencies)
3. All setup commands (ENV, RUN setup scripts)

**Implementation**:
- Modify rocker Dockerfile generation to collect and group operations
- Within each group, maintain deterministic order (alphabetical by extension)

**Benefits**:
- Adding extension only adds new layers, doesn't reorder existing ones
- Similar operations batched together (better for build parallelism)

**Trade-offs**:
- More complex generation logic
- May break extensions with implicit ordering dependencies
- Requires careful testing

**Recommendation**: Consider this after Priority 1 & 2, as it's higher risk.

### Priority 4: Shared Builder Base Stage

**Goal**: Reduce apt package duplication across builders.

**Strategy**: Create a common builder base with frequently-used build tools:
```dockerfile
FROM ubuntu:22.04 AS deps_rocker_builder_base
RUN --mount=type=cache,target=/var/cache/apt,id=apt-cache \
    apt-get update && apt-get install -y \
    curl ca-certificates git \
    build-essential cmake
```

Then builders inherit from this:
```dockerfile
FROM deps_rocker_builder_base AS cargo_builder
# curl and ca-certificates already installed
```

**Benefits**:
- Common tools installed once, cached
- Faster builder stage execution
- Reduced layer duplication

**Trade-offs**:
- All builders now depend on one base (coupling)
- Changes to shared base invalidate all builders
- Need to carefully choose what goes in shared base

**Recommendation**: Experimental. Test with subset of extensions first.

## Implementation Roadmap

### Phase 1: Low-Risk Wins (Recommended for immediate implementation)
1. **Fixed builder base images** (Priority 1)
   - Update SimpleRockerExtension with builder_base_image attribute
   - Update 3-5 stable extensions (cargo, npm, pixi) as proof of concept
   - Document pattern for other extensions
   - Measure cache hit improvement with test scenarios

### Phase 2: Deterministic Ordering (Medium risk)
2. **Alphabetical builder ordering** (Priority 2, Option A)
   - Sort builder stages alphabetically before emission
   - Test that dependency order is preserved in final stage
   - Validate no implicit ordering assumptions break

### Phase 3: Advanced Optimization (Higher risk, consider carefully)
3. **Cache priority system** (Priority 2, Option B)
   - Add cache_priority attribute
   - Assign priorities to existing extensions
   - Implement priority-based sorting

4. **Grouped final stage operations** (Priority 3)
   - Prototype with subset of extensions
   - Extensive testing for ordering dependency issues

### Phase 4: Experimental (Evaluate first)
5. **Shared builder base** (Priority 4)
   - Create proof of concept with 3 extensions
   - Measure build time and cache efficiency
   - Decide if trade-offs are worth it

## Testing Strategy

For each change, measure:
1. **Cache hit rate** when adding/removing/reordering extensions
2. **Build time** for clean builds vs incremental builds
3. **Final image size** (ensure no regression)
4. **Compatibility** across Ubuntu versions for builder artifacts

### Test Scenarios
- Scenario A: Add new extension to existing 10-extension config
- Scenario B: Change final base image (ubuntu:22.04 → 24.04)
- Scenario C: Update extension (change version number)
- Scenario D: Reorder extensions in config file

### Success Metrics
- Cache hit rate for Scenario A: Currently ~30-50%, target >90%
- Cache hit rate for Scenario B: Currently ~0%, target >80%
- Build time improvement: Target 40-60% for common incremental builds

## Risks and Mitigations

### Risk: Breaking existing extension behavior
**Mitigation**:
- Make changes opt-in with backward-compatible defaults
- Extensive CI testing across all extensions
- Document migration path clearly

### Risk: Docker version compatibility
**Mitigation**:
- Test with Docker versions 20.x - 27.x
- Fallback to current behavior if features unavailable
- Document minimum Docker version requirements

### Risk: Complexity increase
**Mitigation**:
- Start with simple changes (Priority 1)
- Document rationale and usage clearly
- Provide examples and migration guide

## Open Questions

1. Should builder_base_image be configurable globally (e.g., in rockerc.yaml)?
2. How to handle extensions that genuinely need specific base image versions?
3. Should cache_priority be exposed to end users or just extension developers?
4. What's the maximum acceptable base image version spread (builder vs final)?

## References

- Docker multi-stage build caching: https://docs.docker.com/build/cache/
- BuildKit cache mounts: https://docs.docker.com/build/cache/backends/
- Related: palanteer_builder_snippet.Dockerfile:2-3 (comments about fixed base)
