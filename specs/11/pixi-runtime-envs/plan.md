# Implementation Plan: Pixi Runtime Environments

## Overview
Transform pixi from build-time environment baking to runtime installation with persistent volumes. This enables pixi's cache optimization while maintaining Docker isolation.

## Architecture Changes

### Current (Build-time)
```
Builder Stage → Install Env → Copy Env → Runtime Image (50k+ files)
```

### New (Runtime)
```
Thin Image (pixi binary only) → Container Start → pixi install (from cache volume) → Activate
```

## Files to Create

### 1. `deps_rocker/extensions/pixi/pixi_entrypoint.sh`
Entrypoint script that:
- Ensures pixi environment exists (idempotent)
- Runs `pixi install --locked` using persistent cache
- Activates environment via shell-hook
- Execs into user command

### 2. `deps_rocker/extensions/pixi/pixi_config.toml.em`
Pixi configuration template for detached environments:
```toml
detached-environments = "/opt/pixi/envs"
```

### 3. `scripts/pixi_runtime_container.py` (helper script)
Container lifecycle manager:
- Compute dependency fingerprint (hash of manifest + lock)
- Check for existing container with matching fingerprint
- Reuse container if match, else create new
- Mount persistent volumes for cache and envs
- Handle container start and attach

## Files to Modify

### 4. `deps_rocker/extensions/pixi/pixi.py`
**Add new mode flag:**
```python
@staticmethod
def register_arguments(parser, defaults=None):
    parser.add_argument(
        "--pixi-runtime-mode",
        action="store_true",
        default=defaults.get("pixi_runtime_mode", False),
        help="Use runtime pixi install instead of baking environment (faster, requires volume mounts)"
    )
    parser.add_argument(
        "--pixi-cache-volume",
        type=str,
        default=defaults.get("pixi_cache_volume", "pixi-cache"),
        help="Docker volume name for pixi cache (runtime mode only)"
    )
```

**Modify snippet generation:**
- When `--pixi-runtime-mode=True`:
  - Skip builder stage (no environment baking)
  - Copy only pixi binary
  - Install pixi config for detached envs
  - Copy entrypoint script
  - Set entrypoint to pixi_entrypoint.sh

- When `--pixi-runtime-mode=False` (default, backward compatible):
  - Keep current multi-stage build behavior
  - Bake environment into image (CI/distribution use case)

### 5. `deps_rocker/extensions/pixi/pixi_runtime_snippet.Dockerfile`
New snippet for runtime mode:
```dockerfile
# Copy pixi binary only
COPY --from=ghcr.io/prefix-dev/pixi:latest /usr/local/bin/pixi /usr/local/bin/pixi

# Configure detached environments
RUN mkdir -p /etc/pixi /opt/pixi/{cache,envs}
COPY pixi_config.toml /etc/pixi/config.toml

# Environment variables
ENV PIXI_CACHE_DIR=/opt/pixi/cache
ENV PIXI_CONFIG_FILE=/etc/pixi/config.toml

# Copy and set entrypoint
COPY pixi_entrypoint.sh /usr/local/bin/pixi_entrypoint.sh
RUN chmod +x /usr/local/bin/pixi_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/pixi_entrypoint.sh"]
```

### 6. `deps_rocker/pixi_env.py`
**Add manifest writing capability:**
```python
def write_manifest_to_path(self, path: Path, ...) -> None:
    """Write generated manifest to filesystem for runtime use"""
    manifest_content = self.render_manifest(...)
    path.write_text(manifest_content)
```

**Add lockfile generation support:**
- Method to generate lockfile via `pixi install` (for reproducibility)
- Hash computation for fingerprinting

### 7. Documentation Updates
**`README.md`:**
- Add "Runtime vs Build-time Modes" section
- Document volume requirements
- Provide usage examples
- Performance comparison

**`CLAUDE.md`:**
- Update pixi extension implementation checklist
- Add runtime mode guidelines

## Implementation Details

### Entrypoint Script Logic
```bash
#!/usr/bin/env bash
set -euo pipefail

export PIXI_CACHE_DIR=/opt/pixi/cache
export PIXI_CONFIG_FILE=/etc/pixi/config.toml

MANIFEST=${PIXI_MANIFEST:-/workspace/.deps-rocker/pixi.toml}
ENV_NAME=${PIXI_ENV_NAME:-tools}

# Install environment if manifest exists
if [ -f "$MANIFEST" ]; then
    install_flags="--environment $ENV_NAME"
    if [ -f "${MANIFEST%.toml}.lock" ]; then
        install_flags="$install_flags --locked"
    fi

    pixi install --manifest-path "$MANIFEST" $install_flags

    # Activate environment
    eval "$(pixi shell-hook --manifest-path "$MANIFEST" --environment "$ENV_NAME")"
fi

# Execute user command
exec "$@"
```

### Container Fingerprinting
Hash components:
- Generated pixi.toml content
- pixi.lock content (if exists)
- Enabled extension list
- Runtime mode flag

Use SHA256 hash as container label: `deps-rocker.pixi-fingerprint=<hash>`

### Volume Management
Two approaches:

**A. Named Volume (simpler)**
```bash
docker volume create pixi-cache
docker run -v pixi-cache:/opt/pixi ...
```

**B. Project-specific (isolated)**
```bash
docker run -v pixi-cache-${PROJECT_HASH}:/opt/pixi ...
```

Recommend option A for max cache sharing across projects.

## Testing Strategy

### 1. Unit Tests
- Test manifest generation with runtime mode flag
- Test config file generation
- Test fingerprint computation

### 2. Integration Tests
Create `test/test_pixi_runtime.py`:
- Build image with runtime mode
- Start container with volumes
- Verify environment installs correctly
- Verify cache reuse on second start
- Verify different manifest = different environment

### 3. Performance Benchmarks
Measure:
- First install (cold cache): baseline
- Second install (warm cache): should be <10s for typical env
- Container reuse (matching fingerprint): <1s attach time

## Migration Path

### Phase 1: Add runtime mode (this PR)
- Default: `pixi_runtime_mode=False` (current behavior)
- Opt-in: `--pixi-runtime-mode` for testing

### Phase 2: Gather feedback
- Test with real projects
- Identify edge cases
- Tune cache strategies

### Phase 3: Future (optional)
- Consider making runtime mode default
- Add auto-detection heuristics
- Integrate with rockerc for seamless volume management

## Backward Compatibility

**Critical requirement:** Existing users see no change unless they opt-in to `--pixi-runtime-mode`.

All current arguments and behavior remain unchanged when runtime mode is disabled.

## Benefits Summary

1. **Speed**: Warm cache installs are 10-100x faster than rebuilding images
2. **Cache sharing**: Common packages shared across all projects using same volume
3. **Dynamic deps**: Change dependencies without rebuilding images
4. **Storage**: Single copy of packages, not per-image duplication
5. **Flexibility**: Easy to switch between environments without new images

## Known Limitations

1. **First install**: Still requires downloading packages (but cached for future)
2. **Volume dependency**: Losing volume = lose cache (but can rebuild)
3. **Complexity**: Slightly more complex setup than pure image-based approach
4. **Rocker integration**: Container management requires manual volume mounting or rockerc enhancement

## Future Enhancements (Out of Scope)

1. Automatic rockerc integration for volume management
2. Cloud-backed cache volumes for team sharing
3. Prewarming cache volumes with common packages
4. Multi-stage with both baked baseline + runtime additions
