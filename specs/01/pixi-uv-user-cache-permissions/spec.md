# Spec: Fix Pixi/UV cache permissions for non-root users

## Goal
Ensure that pixi and uv can create cache directories when running as a non-root user in the container.

## Problem
When using the uv extension via renv, pixi/uv tries to create cache directories at `/home/user/.cache/rattler/cache/uv-cache` but fails with:
```
x failed to create uv cache directory
`-> failed to create directory `/home/user/.cache/rattler/cache/uv-cache`: Permission denied (os error 13)
```

This occurs because:
1. Docker build runs as root
2. Container runs as non-root user (via user extension)
3. Cache directories don't exist or have wrong ownership

## Requirements
- Pixi and UV extensions must create cache directories with proper ownership for the target user
- Cache directories should be created at build time with correct permissions
- Solution must work when user extension is used
- Must handle the case where username is not known at build time

## Solution
- Create cache directories during Docker build
- Set ownership to match the user that will run the container
- Use template variables to get username from user extension

## Out of Scope
- Changing cache mount locations
- Supporting multiple users in same image

## References
- User extension for username template variable
- Pixi cache directory: ~/.cache/pixi and ~/.cache/rattler
- UV cache directory: ~/.cache/uv
