# Remove isaac_sim extension and investigate disk space issues

CI is failing on GitHub when testing all extensions together due to disk space exhaustion.

## Actions

1. **Remove isaac_sim extension** - Not widely used, complex dependencies
2. **Investigate disk space usage** - Identify what's causing the sudden space issues

## Potential causes of disk space issues:
- Docker layer caching with BuildKit
- Multiple large builder stages running in parallel
- Pixi cache growing during builds
- Docker images not being cleaned up properly

## Changes

1. Delete isaac_sim extension directory
2. Remove from pyproject.toml entry points
3. Check Docker cache usage patterns
4. Verify tests don't accumulate unused images
