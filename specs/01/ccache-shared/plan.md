# Shared ccache Implementation Plan

## Current State
- No ccache support in deps_rocker
- Need to enable compiler cache sharing between host and containers
- SimpleRockerExtension provides infrastructure for volume mounting via `get_docker_args()`

## Implementation Steps

### 1. Create Extension Directory Structure
- Create `deps_rocker/extensions/ccache/` directory
- Create `__init__.py` to export the extension class
- Create `ccache.py` with the main extension class
- Create `ccache_snippet.Dockerfile` for Docker configuration
- Create `test.sh` for extension testing

### 2. Implement Extension Class
- Inherit from `SimpleRockerExtension`
- Set `name = "ccache"`
- Set `apt_packages = ["ccache"]` to install ccache
- Override `get_docker_args()` to mount host ccache directory
- Add environment variable configuration in snippet

### 3. Docker Volume Mounting Strategy
- Host ccache directory: `~/.ccache`
- Container ccache directory: `/root/.ccache` (following nvim pattern)
- Create host directory if it doesn't exist
- Mount with `-v` flag in `get_docker_args()`

### 4. Environment Configuration
- Set `ENV CCACHE_DIR=/root/.ccache` in Dockerfile snippet
- This ensures ccache uses the mounted directory

### 5. Entry Points
- Add to `pyproject.toml`: `ccache = "deps_rocker.extensions.ccache.ccache:Ccache"`

### 6. Testing
- Add `ccache` to `EXTENSIONS_TO_TEST` in `test/test_extensions_generic.py`
- Create `test.sh` to verify:
  - ccache is installed
  - ccache command works
  - CCACHE_DIR is set correctly

## Technical Details

### Volume Mount
```python
def get_docker_args(self, cliargs):
    from pathlib import Path

    ccache_dir = Path.home() / ".ccache"
    ccache_dir.mkdir(exist_ok=True)  # Create if doesn't exist

    return f" -v {ccache_dir}:/root/.ccache"
```

### Dockerfile Snippet
```dockerfile
# ccache is installed via apt_packages
ENV CCACHE_DIR=/root/.ccache
```

## Benefits
- Persistent compilation cache across container rebuilds
- Shared cache between multiple containers
- Faster C/C++ compilation times
- Zero cache warm-up time for repeated builds
