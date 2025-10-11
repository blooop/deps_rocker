# VCS Tool Repository Caching

## Objective
Add BuildKit cache mounts to vcstool extension to cache cloned repositories, avoiding repeated git clones on rebuilds.

## Scope
- Update `deps_rocker/extensions/vcstool/vcstool_snippet.Dockerfile` to cache the vcs import results per manifest
- Document the pattern in `CLAUDE.md` for future extensions

## Implementation
Use BuildKit cache mount to store imported repositories, then copy to final destination. This allows Docker to reuse cached git clones across builds.

## Benefits
- Avoid re-cloning repositories on every build
- Significant speedup for workspaces with many/large repos
- Reduced network bandwidth usage
- Safer than git reference repositories (no corruption risk)

# Plan: VCS Tool Repository Caching

## Background
Currently, vcstool runs `vcs import` directly to the final destination on every Docker build. This means all repositories are cloned fresh each time, which is slow and bandwidth-intensive for workspaces with many dependencies.

Following the pattern from palanteer extension (which caches git clones), we can cache the vcs import output and copy it to the final destination.

## Current Implementation
```dockerfile
RUN mkdir -p @(dependencies_root)/@(dep["path"]) && \
    vcs import --recursive @(dependencies_root)/@(dep["path"]) < @(repos_root)/@(dep["dep"])
```

## Proposed Implementation
```dockerfile
RUN --mount=type=cache,target=/root/.cache/vcs-repos,id=vcs-repos-cache \
    mkdir -p /root/.cache/vcs-repos/@(dep["path"]) && \
    vcs import --recursive /root/.cache/vcs-repos/@(dep["path"]) < @(repos_root)/@(dep["dep"]) && \
    mkdir -p @(dependencies_root)/@(dep["path"]) && \
    cp -r /root/.cache/vcs-repos/@(dep["path"])/* @(dependencies_root)/@(dep["path"])/ || true
```

## Implementation Steps
1. Update vcstool_snippet.Dockerfile with cache mount
2. Import to cache directory first
3. Copy from cache to final destination
4. Add `|| true` to handle empty directories gracefully

## Testing
- Run `pixi run ci` to verify builds succeed
- Test with actual .repos files to ensure repositories are cloned correctly
- Verify subsequent builds reuse cached repositories

## Notes
- Cache is per-manifest path using `@(dep["path"])` to avoid conflicts
- Uses same cache ID `vcs-repos-cache` for all repos to share the cache mount
- The `|| true` handles cases where vcs import creates no output
- Follows BuildKit cache mount best practices from CLAUDE.md
