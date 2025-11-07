# Implementation Plan: Palanteer Builder Pattern

## 1. Update palanteer.py
- Add `builder_apt_packages` attribute with build-time dependencies:
  - build-essential
  - cmake
  - python3-dev
  - libgl1-mesa-dev
  - libglu1-mesa-dev
  - libx11-dev
  - libxrandr-dev
  - libxinerama-dev
  - libxcursor-dev
  - libxi-dev
- Keep only runtime dependencies in `apt_packages` (if any are needed)
- These dependencies are currently all for building

## 2. Create palanteer_builder_snippet.Dockerfile
This will contain:
- FROM ubuntu:24.04 AS palanteer_builder (FIXED base image for global cache sharing)
- Cache mount for git repository (already exists in current implementation)
- Cache mount for build artifacts (NEW)
- Clone/update palanteer repository
- Compile with cmake and make
- Place binaries in builder output directory (`/opt/deps_rocker/palanteer`)

Key improvements:
- Cache the compiled binaries, not just the source
- Build artifacts persist across Docker builds
- Only rebuild when source changes
- **Fixed builder base image ensures cache sharing across ALL projects** (doesn't matter what final image you're building)

## 3. Update palanteer_snippet.Dockerfile
Simplify to:
- COPY binaries from builder stage
- Set PATH if needed

## 4. Testing Strategy
- Build a Docker image with palanteer extension
- Verify binaries are installed and functional
- Build again without changes to verify caching works
- Run existing test.sh to ensure functionality

## Pattern Reference
Following the same approach as:
- `lazygit`: Downloads binary and caches it
- `nvim`: Downloads binary and caches it
- `urdf_viz`: Downloads binary and caches it
- But for palanteer we compile from source like some Rust projects

The key insight: Cache the *compiled artifacts*, not just the source code.
