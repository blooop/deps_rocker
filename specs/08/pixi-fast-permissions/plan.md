# Implementation Plan: Optimize Pixi Build Performance

## Analysis
Current bottlenecks in `deps_rocker/extensions/pixi/pixi_snippet.Dockerfile`:
```dockerfile
COPY --from=builder /opt/build/pixi/.pixi /opt/pixi/.pixi      # ← COPIES TOO MUCH
COPY --from=builder /opt/build/pixi/pixi.toml /opt/pixi/pixi.toml
COPY --from=builder /opt/build/pixi/pixi.lock /opt/pixi/pixi.lock
RUN chmod -R a+rx /opt/pixi  # ← SLOW: walks entire tree
```

Investigation shows:
- `.pixi/envs/` = 352MB, ~9k files (NEEDED: contains installed packages)
- `.pixi/task-cache-v0/` = 16KB (NOT NEEDED: build-time cache)
- Package download cache already uses BuildKit cache mounts (good!)
- But we copy the entire `.pixi/` with all its metadata

## Implementation Steps

### 1. Update pixi_snippet.Dockerfile

**For the pixi binary** (line 4):
- Change to: `COPY --chmod=755 --from=... /usr/local/bin/pixi`
- Single file, needs execute permission

**For the environment** (line 5):
- OLD: `COPY --from=... .pixi {runtime}/.pixi` (copies everything)
- NEW: `COPY --chmod=755 --from=... .pixi/envs/{env_name} {runtime}/.pixi/envs/{env_name}`
- Only copies the specific environment, not task-cache or other envs
- chmod 755 = rwxr-xr-x (owner rwx, group/other rx)
- Directories get execute (traversable), files readable, executables keep +x

**For pixi.toml and pixi.lock** (lines 6-7):
- Change to: `COPY --chmod=644`
- Config files, just need read permission

**Remove** the separate `RUN chmod -R` command (old line 9)

### 2. Why This Works

**COPY --chmod optimization:**
- `--chmod=755` on directories: readable/traversable by all users
- Files inside inherit from source permissions during copy
- Executables already have +x from builder, preserving that
- Much faster: permissions set during copy, not in separate walk
- BuildKit optimization: permission setting is part of copy operation

**Copying only specific environment:**
- Reduces copy size: only ~9k files for one env vs entire `.pixi/` tree
- Avoids copying build artifacts (task-cache-v0/)
- Still provides all runtime binaries/libraries needed
- PATH already points to specific env: `.pixi/envs/{env_name}/bin`

### 3. Testing
Verify in tests that:
- Tools in PATH are executable
- Works with rocker's user mapping (non-root UID)
- Build time is significantly faster

### 4. Files to Modify
- `deps_rocker/extensions/pixi/pixi_snippet.Dockerfile`

## Alternative Considered: umask in builder
Could set `umask 022` in builder stage before install, but that requires modifying builder snippet and doesn't help with files already copied. The COPY --chmod approach is simpler and more direct.

## Risk Assessment
Low risk:
- COPY --chmod is well-supported in modern BuildKit
- Permissions (755 for dirs, 644 for files) are standard
- Worst case: can revert to chmod -R if issues arise
- Tests will catch permission problems before merge
