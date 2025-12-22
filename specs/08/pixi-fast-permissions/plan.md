# Implementation Plan: Optimize Pixi Permission Setting

## Analysis
Current bottleneck in `deps_rocker/extensions/pixi/pixi_snippet.Dockerfile`:
```dockerfile
COPY --from=builder /opt/build/pixi/.pixi /opt/pixi/.pixi
COPY --from=builder /opt/build/pixi/pixi.toml /opt/pixi/pixi.toml
COPY --from=builder /opt/build/pixi/pixi.lock /opt/pixi/pixi.lock
RUN chmod -R a+rx /opt/pixi  # ← SLOW: walks entire tree
```

The `.pixi` directory contains the full conda environment with tens of thousands of files.

## Implementation Steps

### 1. Update pixi_snippet.Dockerfile
Replace the COPY + chmod pattern with COPY --chmod:

**For the pixi binary** (line 4):
- Already a single file, just needs execute permission
- Change to: `COPY --chmod=755`

**For the .pixi directory** (line 5):
- Contains the environment with thousands of files/dirs
- Change to: `COPY --chmod=755`
- Note: chmod 755 = rwxr-xr-x (owner rwx, group/other rx)
- Directories get execute (traversable), files get read, executables keep their +x

**For pixi.toml and pixi.lock** (lines 6-7):
- Config files, just need read permission
- Change to: `COPY --chmod=644`

**Remove** the separate `RUN chmod -R` command (line 9)

### 2. Why This Works
- `--chmod=755` on directories: makes them readable/traversable by all users
- Files inside inherit from their source permissions during copy
- Executables already have +x from the builder stage, preserving that
- Much faster: permissions set during copy, not after
- BuildKit optimization: permission setting is part of copy operation

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
