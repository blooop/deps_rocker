# Builder Stage Cache Efficiency Review

## Summary

Review of builder stage implementation to identify cache inefficiencies when changing loaded extensions.

## Current Architecture

### Dockerfile Generation Flow
1. `get_preamble()` → All builder stages (FROM base_image AS {name}_builder)
2. `get_snippet()` → Final stage with COPY --from={builder_stage} + runtime setup
3. `get_user_snippet()` → User-specific configuration

### Extension Ordering
- Extensions ordered by dependency graph (`depends_on_extension`, `invoke_after()`)
- Builder stages emitted in extension invocation order
- Final stage COPYs and commands emitted in same order

## Cache Inefficiencies Identified

### 1. Builder Stage Base Image Coupling
**Issue**: All builder stages use `FROM {base_image}` where base_image is the final image base.

**Impact**: Changing the final base image (e.g., ubuntu:22.04 → ubuntu:24.04) invalidates ALL builder stages, even though:
- Compiled binaries often work across Ubuntu versions (compatible glibc)
- Downloaded tools (npm, cargo installers) are base-image independent
- Build tools (cmake, gcc) could use a fixed builder base

**Example**: Palanteer builder comments acknowledge this but doesn't implement it:
```dockerfile
# Use a fixed base image for the builder to maximize cache sharing across all projects
FROM {base_image} AS palanteer_builder  # Still uses variable base
```

### 2. Extension Order Changes Break Cache
**Issue**: Adding/removing extensions changes builder stage order in the Dockerfile.

**Impact**: Docker caches layers sequentially. If extension B is added before extension A:
- Old: `FROM base AS A_builder` (line 10) → `FROM base AS C_builder` (line 30)
- New: `FROM base AS B_builder` (line 10) → `FROM base AS A_builder` (line 30) → `FROM base AS C_builder` (line 50)
- Even though A and C haven't changed, their cache is invalidated due to line number shift

**Root cause**: Docker layer cache is position-dependent in multi-stage builds.

### 3. Final Stage Command Ordering
**Issue**: In the final stage, COPY commands from builder stages are interspersed with apt installs and other commands.

**Impact**: Changing extension order changes when COPYs occur:
```dockerfile
# Extension A enabled
RUN apt-get install A-runtime-deps
COPY --from=A_builder /opt/A /usr/local/A

# Extension B added
RUN apt-get install B-runtime-deps  # Cache break for everything after
COPY --from=B_builder /opt/B /usr/local/B
RUN apt-get install A-runtime-deps  # Reordered, cache miss
COPY --from=A_builder /opt/A /usr/local/A
```

### 4. Apt Package Duplication Across Builders
**Issue**: Common builder packages (curl, git, ca-certificates) reinstalled per builder stage.

**Mitigation**: BuildKit apt cache mounts reduce this impact, but still inefficient.

**Example**:
- cargo_builder: `apt-get install curl ca-certificates`
- npm_builder: `apt-get install curl ca-certificates git`
- Each builder reinstalls these packages from scratch (layer-wise)

## Cache Efficiency Strengths

### 1. BuildKit Cache Mounts
- Persistent across builds: `/var/cache/apt`, git repos, download caches
- Smart strategies: Palanteer checks commit hash before rebuilding
- Proper cache namespacing: `id=nvm-git-cache`, `id=cargo-rustup-cache`

### 2. Standardized Output Paths
- All builders output to `/opt/deps_rocker/{name}/`
- Predictable COPY paths regardless of builder changes
- Enables consistent layer hashing within each extension

### 3. Separation of Concerns
- `builder_apt_packages` vs `apt_packages` clearly separates build/runtime deps
- Build artifacts cleanly isolated from final image
- Git repos and build tools never reach final image

## Quantified Impact

### Scenario: Add new extension "tool_x" to project using 10 existing extensions

**Current behavior**:
- If tool_x is alphabetically early or has few dependencies: Breaks cache for 7-10 extensions
- If tool_x is late in order: Breaks cache for 0-3 extensions
- Unpredictable based on dependency graph resolution

**Ideal behavior**:
- Only tool_x builder and final stage commands should be new layers
- Existing 10 extensions should be cache hits
- Cache breaks should be deterministic and minimal
