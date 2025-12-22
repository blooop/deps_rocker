# Implementation Plan: NPM BuildKit Cache Mounts

## Files to Modify

### 1. deps_rocker/extensions/gemini/gemini_user_snippet.Dockerfile
- Add BuildKit cache mount to the npm install command
- Cache directory: `/root/.npm`
- Cache ID: `npm-cache` (shared with other npm extensions)

### 2. deps_rocker/extensions/codex/codex_user_snippet.Dockerfile
- Add BuildKit cache mount to the npm install command
- Cache directory: `/root/.npm`
- Cache ID: `npm-cache` (shared with other npm extensions)

## Implementation Details

### Cache Mount Syntax
```dockerfile
RUN --mount=type=cache,target=/root/.npm,id=npm-cache \
    npm install -g @package/name
```

### Why This Works
- NPM stores downloaded package tarballs in `/root/.npm/_cacache/`
- On subsequent builds, npm checks cache before downloading
- BuildKit cache mount preserves this directory across builds
- Cache is shared across all builds using the same cache ID
- Cache is not included in final image layers

### Testing
- Build with gemini extension, verify packages download
- Rebuild, verify faster installation (npm cache hit messages)
- Build with codex extension, verify shared cache benefits
- Confirm image size unchanged

## Benefits
- Faster builds after first run
- Shared cache across projects
- No impact on image size
- Standard BuildKit practice
