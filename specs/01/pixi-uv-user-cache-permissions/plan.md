# Plan: Fix Pixi/UV cache permissions for non-root users

## Root Cause Analysis
The issue occurs when:
1. Pixi/UV extensions are used in combination with the user extension
2. The Docker build runs as root
3. The container runs as a non-root user
4. Pixi tries to create cache directories at `/home/{username}/.cache/rattler/cache/uv-cache`
5. The parent directory doesn't exist or is owned by root

## Implementation Steps

### 1. Update Pixi Extension
- Modify `pixi_snippet.Dockerfile` to create cache directories
- Use template variable to get username from user extension dependencies
- Create directories: `.cache/pixi` and `.cache/rattler`
- Set ownership using chown

### 2. Update UV Extension
- Modify `uv_snippet.Dockerfile` to create cache directories
- Add dependency on user extension
- Create directory: `.cache/uv`
- Set ownership using chown

### 3. Handle Edge Cases
- Check if user extension is being used (username available)
- If no username, skip cache directory creation (build-time cache only)
- Ensure directories are created with proper permissions (755 for directories)

## Technical Details

### Template Variables Needed
- `name`: Username from user extension (already available in extensions that depend on user)

### Dockerfile Changes
```dockerfile
# In pixi_snippet.Dockerfile
RUN if [ -n "@(name)" ]; then \
    mkdir -p /home/@(name)/.cache/pixi /home/@(name)/.cache/rattler && \
    chown -R @(name):@(name) /home/@(name)/.cache; \
    fi
```

### Dependencies
- Pixi extension must depend on user extension
- UV extension must depend on user extension
- Both already have this dependency, just need to use the template variable

## Testing
- Test with renv setup that triggers the error
- Verify cache directories are created with correct ownership
- Run pixi run ci to ensure no regressions
