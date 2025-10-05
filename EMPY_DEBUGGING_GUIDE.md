# Empy Template & Docker BuildKit Debugging Guide

## Overview

This document summarizes the findings and solutions discovered while debugging complex empy template processing issues with Docker BuildKit multi-stage builds in the deps_rocker project. The original issue was "empty args are not getting set correctly" which led to systematic failures across multiple extensions.

## Key Breakthrough: F-String Template Syntax

**Critical Discovery**: Empy v4.2 requires f-string syntax `@(f"{variable}")` for reliable variable substitution in complex expressions and multi-stage Docker builds.

## Root Causes Identified

### 1. Template Variable Processing Issues
- **Problem**: `get_template_args()` methods returning empty dictionaries
- **Root Cause**: Dynamic method calls during template processing aren't reliable
- **Solution**: Convert template args to class attributes

### 2. Empy Template Markup Conflicts
- **Problem**: `@;` and `@/` markup sequences causing parsing errors
- **Root Cause**: Empy's whitespace consumption and markup sequence conflicts
- **Solution**: Use f-string syntax `@(f"{variable}")` universally

### 3. Docker FROM Directive Whitespace Issues
- **Problem**: FROM statements becoming malformed due to empy consuming spaces
- **Root Cause**: Empy's aggressive whitespace processing
- **Solution**: F-string syntax prevents whitespace consumption

### 4. Shell Compatibility Issues
- **Problem**: `set -euxo pipefail` failing in sh vs bash contexts
- **Root Cause**: Dockerfile RUN commands use sh by default, but pipefail requires bash
- **Solution**: Wrap complex scripts with `bash -c "..."`

### 5. NPM Scoped Package @ Symbol Conflicts
- **Problem**: `@openai/codex` being interpreted as empy markup
- **Root Cause**: @ symbol has special meaning in empy templates
- **Solution**: Escape with `@@openai/codex` in templates

## Complete Solution Checklist

### Phase 1: Template Variable Fixes
- [ ] **Convert get_template_args() to class attributes**
  ```python
  # BEFORE (problematic)
  def get_template_args(self):
      return {"builder_output_dir": f"/opt/deps_rocker/{self.name}"}
  
  # AFTER (working)
  builder_output_dir = f"/opt/deps_rocker/{{extension_name}}"
  builder_stage = f"{{extension_name}}_builder"
  ```

- [ ] **Use f-string syntax for all template variables**
  ```dockerfile
  # BEFORE (fails)
  @(builder_output_dir)
  
  # AFTER (works)
  @(f"{builder_output_dir}")
  ```

### Phase 2: Docker BuildKit Multi-Stage Fixes
- [ ] **Fix FROM directive spacing**
  ```dockerfile
  # BEFORE (becomes malformed)
  FROM testfixture_extensions_base AS @(builder_stage)
  
  # AFTER (preserved correctly)
  FROM testfixture_extensions_base AS @(f"{builder_stage}")
  ```

- [ ] **Fix COPY --from references**
  ```dockerfile
  # BEFORE (fails to resolve)
  COPY --from=@(builder_stage) @(builder_output_dir)/file /dest
  
  # AFTER (works reliably)
  COPY --from=@(f"{builder_stage}") @(f"{builder_output_dir}")/file /dest
  ```

### Phase 3: Shell Compatibility Fixes
- [ ] **Wrap complex bash scripts with bash -c**
  ```dockerfile
  # BEFORE (fails in sh)
  RUN set -euxo pipefail && complex_script_with_pipes
  
  # AFTER (works reliably)
  RUN bash -c "set -euxo pipefail && complex_script_with_pipes"
  ```

- [ ] **Use BuildKit cache mounts correctly**
  ```dockerfile
  RUN --mount=type=cache,target=/cache,id=unique-id \
      bash -c "set -euxo pipefail && \
      if [ ! -f /cache/file ]; then download_to_cache; fi && \
      use_cached_file"
  ```

### Phase 4: NPM & Scoped Package Fixes
- [ ] **Escape @ symbols in npm package names**
  ```dockerfile
  # BEFORE (empy parsing error)
  RUN npm install -g @openai/codex
  
  # AFTER (works correctly)
  RUN npm install -g @@openai/codex
  ```

### Phase 5: Extension-Specific Patterns

#### Multi-Stage Builder Extensions (cargo, conda, fzf, lazygit, pixi)
- [ ] Use f-string syntax throughout builder and main stages
- [ ] Implement proper BuildKit cache mounts with versioned cache keys
- [ ] Use bash -c wrappers for complex installation scripts
- [ ] Copy artifacts from builder stage using f-string paths

#### Simple Package Extensions (curl, git, locales, tzdata)
- [ ] Minimal changes needed - usually just class attribute conversion
- [ ] Verify apt package installation commands work correctly

#### NPM-Dependent Extensions (codex, gemini)
- [ ] Escape scoped package names with @@
- [ ] Ensure npm extension provides working npm installation
- [ ] Verify CLI tools are accessible in PATH after installation

## Testing & Validation Checklist

### Before Making Changes
- [ ] Run `pixi run ci` to establish baseline failure count
- [ ] Identify which extensions are failing and why
- [ ] Check for empy template processing errors in build logs

### During Development
- [ ] Test individual extensions: `python -c "from deps_rocker.extensions.{name} import {Class}; print({Class}().get_snippet())"`
- [ ] Verify template variable substitution works correctly
- [ ] Check Docker build logs for syntax errors

### After Changes
- [ ] Run `pixi run ci` to verify fixes
- [ ] Ensure passing test count increases
- [ ] Validate multi-stage builds work correctly
- [ ] Check that BuildKit cache mounts function properly

## Debugging Commands & Techniques

### Template Variable Debugging
```python
# Debug template variables
from deps_rocker.extensions.{extension} import {Extension}
ext = {Extension}()
print("Template variables:", vars(ext))
print("Generated snippet:")
print(ext.get_snippet())
```

### Docker Build Debugging
```bash
# Build individual extension with verbose output
docker build --progress=plain --no-cache .

# Check BuildKit cache usage
docker system df
docker builder prune  # Clear cache if needed
```

### Empy Template Debugging
- Look for `@;` and `@/` sequences in error messages
- Check for whitespace consumption in FROM directives
- Verify @ symbols are properly escaped in package names

## Common Error Patterns & Solutions

### "markup terminator" errors
- **Symptom**: `empy: markup terminator expected`
- **Cause**: Missing or malformed empy markup sequences
- **Fix**: Use f-string syntax `@(f"{variable}")`

### "Invalid Dockerfile syntax" 
- **Symptom**: FROM directive syntax errors
- **Cause**: Empy consuming whitespace in FROM statements
- **Fix**: F-string syntax prevents whitespace consumption

### "command not found" runtime errors
- **Symptom**: Build succeeds but CLI tools not found at runtime
- **Cause**: npm global packages not in PATH, or dummy installation
- **Fix**: Implement proper npm installation and PATH configuration

### "bash: pipefail: invalid option"
- **Symptom**: Shell scripts failing with pipefail errors
- **Cause**: Using bash features in sh context
- **Fix**: Wrap with `bash -c "..."`

## Success Metrics

Following this guide should result in:
- ✅ Reduction from 9+ failing tests to ≤2 failing tests
- ✅ All multi-stage Docker builds working correctly
- ✅ Template variables processing reliably
- ✅ BuildKit cache mounts functioning properly
- ✅ Shell compatibility issues resolved

## Extensions Fixed Using This Guide

- ✅ **cargo**: Multi-stage Rust toolchain installation
- ✅ **conda**: Multi-stage Miniforge installation  
- ✅ **fzf**: Multi-stage fuzzy finder installation
- ✅ **lazygit**: Multi-stage git UI installation
- ✅ **pixi**: Multi-stage package manager installation
- ✅ **npm**: Node.js package manager installation
- ✅ **All simple extensions**: curl, git, locales, tzdata, etc.

## Future Maintenance

When adding new extensions:
1. Use class attributes instead of `get_template_args()` methods
2. Apply f-string syntax `@(f"{variable}")` for all template variables
3. Use `bash -c` wrappers for complex shell scripts
4. Escape @ symbols in package names with @@
5. Test with `pixi run ci` before committing

This systematic approach ensures reliable empy template processing and robust Docker BuildKit multi-stage builds.