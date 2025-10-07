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
