# Palanteer Builder Pattern

## Goal
Refactor the palanteer extension to use a builder pattern to cache compilation artifacts and avoid rebuilding on every Docker image build.

## Current Issue
The palanteer extension currently compiles from source in every Docker build, which is time-consuming. While the git repository is cached, the compilation step (cmake + make) runs every time.

## Solution
Use a multi-stage Dockerfile builder pattern where:
1. **Builder stage** (`palanteer_builder_snippet.Dockerfile`): Compiles palanteer once and caches the build artifacts
2. **Main stage** (`palanteer_snippet.Dockerfile`): Simply copies the pre-compiled binaries from the builder stage

## Benefits
- Faster builds when palanteer hasn't changed
- Cleaner separation between build dependencies and runtime dependencies
- Build artifacts are cached across Docker builds
- Follows the established pattern used by other extensions (lazygit, urdf_viz, nvim)

## Implementation Details
- Separate build-time and runtime dependencies:
  - `builder_apt_packages`: Build tools (cmake, gcc) + library headers (-dev packages)
  - `apt_packages`: Runtime libraries only (no -dev packages, smaller footprint)
- Cache both the git repository AND the compiled binaries
- Copy compiled binaries from builder stage to final image
