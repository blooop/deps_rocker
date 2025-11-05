This project uses pixi to manage its environment.

look at the pyproject.toml to see the pixi tasks

Workflow:
    * On first message:
        - analyze the request and determine which category it belongs to (see Specification Categories below)
        - find the highest existing spec number by running: ls -d specs/*/ | grep -oP 'specs/\K\d+' | sort -n | tail -1
        - create a new specification at specs/NN-short-spec-name/spec.md where NN is highest+1, zero-padded to 2 digits
        - if the spec clearly fits into an existing category, consider mentioning the category in the spec name or grouping context
        - keep spec.md as concise as possible
        - create a plan.md in the same folder with expanded implementation details
        - commit the contents of this folder only

    * Every time I ask for a change
        - update the spec.md with clarifications while keeping it concise. commit if there are changes
        - implement the change
        - run `pixi run ci`
        - fix errors and iterate until ci passes
        - only if ci passes commit the changes.

## Specification Categories

When creating a new specification, determine which category it belongs to. This helps organize related work and provides context for future agents.

**cache-optimization**: Performance improvements via caching
- BuildKit cache mounts, pip/pixi/vcstool caching, ccache integration
- Example: buildkit-cache-mount, pip-buildkit-cache, vcstool-repo-cache

**auto-detection**: Automatic extension and dependency detection
- Workspace scanning, file pattern detection, auto-enabling extensions
- Example: auto, auto-detect-llm-extensions, pixi-auto-detect

**ros-integration**: ROS/ROS2 workspace and dependency management
- ROS installation, underlay/overlay setup, vcstool integration, workspace layout
- Example: ros-jazzy-unified, ros-underlay-builder, ros-workspace-layout

**tool-extensions**: Individual development tool installations
- Installing specific tools (editors, language tools, utilities)
- Example: nvim, npm, uv, deps-dev

**build-optimization**: Docker build performance and structure
- Multi-stage builds, layer optimization, build strategies
- Example: multi-stage-builds

**bug-fixes**: Specific fixes for broken functionality
- Addressing errors, correcting invalid syntax, fixing edge cases
- Example: neovim-version-pin, palanteer-dockerfile-copy-fix

**workflow**: Meta-improvements to development process
- Changes to AGENTS.md, CI/CD improvements, project organization
- Example: auto-number-specs

If a specification spans multiple categories or doesn't fit neatly into one, use your best judgment or create a new category as needed.

# Claude Instructions for deps_rocker

This file contains instructions for Claude when working with the deps_rocker project.

## Extension Implementation Checklist

When a user asks to implement a new extension, follow this checklist:

### 1. **Create Extension Directory Structure**
- [ ] Create directory: `deps_rocker/extensions/{extension_name}/`
- [ ] Create `__init__.py` with proper imports
- [ ] Create main extension file: `{extension_name}.py`

### 2. **Implement Extension Class**
- [ ] Inherit from `SimpleRockerExtension`
- [ ] Set proper `name` attribute
- [ ] Add descriptive docstring (used for CLI help)
- [ ] Define `depends_on_extension` tuple if dependencies are needed
- [ ] Override methods like `required()` or `invoke_after()` if needed

### 3. **Create Docker Installation Script**
- [ ] Create `{extension_name}_snippet.Dockerfile` with installation commands
- [ ] Follow Docker best practices (minimize layers, clean up apt cache, etc.)
- [ ] Use appropriate base commands for the tool/language being installed
- [ ] Set proper environment variables if needed

### 4. **Add to Entry Points**
- [ ] Update `pyproject.toml` in `[project.entry-points."rocker.extensions"]` section
- [ ] Use format: `{extension_name} = "deps_rocker.extensions.{extension_name}.{extension_name}:{ClassName}"`

### 5. **Add Tests**
- [ ] Add extension name to `EXTENSIONS_TO_TEST` list in `test/test_extensions_generic.py`
- [ ] Add individual test method: `test_{extension_name}_extension(self)`
- [ ] Create `test.sh` script in extension directory if custom testing is needed
- [ ] Make test script executable: `chmod +x test.sh`
- [ ] Test script should verify the tool was installed and is working

### 6. **Test Script Requirements**
- [ ] Start with proper shebang (#!/bin/bash)
- [ ] Use `set -e` for fail-fast behavior
- [ ] Test that commands are available (`command -v {tool}`)
- [ ] Test basic functionality of the installed tool
- [ ] Echo success message at the end

### 7. **Documentation**
- [ ] Update README.md if the extension adds significant functionality
- [ ] Add example usage in docstring or separate documentation

## Example Extension Structure

```
deps_rocker/extensions/npm/
├── __init__.py                    # Import and expose main class
├── npm.py                         # Main extension class
├── npm_snippet.Dockerfile         # Docker installation commands
└── test.sh                        # Test script 
```

## Common Patterns

### Extension Class Template
```python
from deps_rocker.simple_rocker_extension import SimpleRockerExtension

class MyTool(SimpleRockerExtension):
    """Install my tool for development"""

    name = "my_tool"
    depends_on_extension = ("curl",)  # If dependencies needed
```

### Test Script Template
```bash
#!/bin/bash
set -e

echo "Testing my_tool installation..."

if ! command -v my_tool &> /dev/null; then
    echo "ERROR: my_tool command not found"
    exit 1
fi

# Test basic functionality
my_tool --version

echo "my_tool extension test completed successfully!"
```

## Dependencies

Common dependencies that extensions might need:
- `curl` - For downloading files
- `git` - For cloning repositories
- `locales` - For proper locale support
- `tzdata` - For timezone data

Always check if the tool being installed has specific requirements and add appropriate dependencies.

### BuildKit Cache Mounts
- Use `RUN --mount=type=cache,target=/path/to/cache` for any download or clone that should be reused between builds.
- Inside the `RUN` block, create the cache directory and skip network downloads when the expected file already exists (`if [ ! -f "${cache_file}" ]; then curl ...; fi`).
- Prefer versioned filenames (e.g. include release tag or tool version) so new releases trigger a fresh download while older layers stay cached.
- Keep artifacts (installers, archives, repos) in the cache; do not delete them at the end of the step so future builds can reuse them.
- When cloning repositories, reuse the cached checkout and update it in-place with `git fetch`/`git reset` before copying into the container filesystem.

#### Caching Multi-Repository Imports (vcstool pattern)
When importing multiple repositories (e.g., with vcstool), use a two-step approach:
1. Import to a cache directory first
2. Copy from cache to final destination

```dockerfile
RUN --mount=type=cache,target=/root/.cache/vcs-repos,id=vcs-repos-cache \
    mkdir -p /root/.cache/vcs-repos/@(dep["path"]) && \
    vcs import --recursive /root/.cache/vcs-repos/@(dep["path"]) < @(repos_root)/@(dep["dep"]) && \
    mkdir -p @(dependencies_root)/@(dep["path"]) && \
    cp -r /root/.cache/vcs-repos/@(dep["path"])/. @(dependencies_root)/@(dep["path"])/
```

**Benefits:**
- Repository clones are cached and reused across builds
- Avoids repeated network downloads for the same repos
- Much faster than re-cloning on every build
- Safer than git reference repositories (no corruption risk)

**Notes:**
- Use `/.` instead of `/*` to handle empty directories while still catching real copy errors
- Cache is per-path to avoid conflicts between different manifests
- All repos share the same cache ID to maximize reuse
