# Spec 09: Unconditional Pixi BuildKit Cache Mounts

## Problem
Pixi builds are slow on subsequent runs because cache mounts are conditional on `DOCKER_BUILDKIT=1` being detected at config time. This can fail if the environment variable is set later during build execution.

## Root Cause
- Cache mounts are conditionally generated based on `is_buildkit_enabled()` check during extension configuration
- This check may return False even though BuildKit will be available during the actual build
- Without cache mounts, every build re-downloads and re-solves Pixi dependencies

## Requirements
- Make BuildKit cache mounts unconditional (always present in Dockerfile)
- Assume BuildKit is available (as documented in project goals)
- Ensure cache IDs are stable and reusable across different projects
- Keep final images small (no caches in runtime layers)

## Solution
- Remove conditional cache mount generation
- Always include `--mount=type=cache` directives in builder stages
- Use stable cache IDs: `pixi-cache` and `rattler-cache`
- Document that BuildKit is required

## Success Criteria
- First build of Project A is slow (acceptable)
- Second build of same project is fast
- First build of Project B with overlapping Pixi deps is fast
- Final image size unchanged
