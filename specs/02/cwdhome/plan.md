# CWD Home Extension Implementation Plan

## Overview
Implement an extension that mounts the current working directory into the container's home directory and sets it as the working directory.

## Steps

### 1. Research Existing Extensions
- Look at `cwd` extension to understand volume mounting
- Check how `user` extension handles home directory
- Understand how to set working directory in containers

### 2. Create Extension Structure
- Create `deps_rocker/extensions/cwdhome/` directory
- Create `__init__.py` with proper imports
- Create `cwdhome.py` with main class

### 3. Implement Extension Class
- Inherit from `SimpleRockerExtension`
- Set `name = "cwdhome"`
- Override `get_snippet()` to set WORKDIR
- Override volume mounting to map CWD -> home directory
- May need to depend on `user` extension to ensure home directory exists

### 4. Add Entry Point
- Update `pyproject.toml` entry points section

### 5. Add Tests
- Add to `EXTENSIONS_TO_TEST` in test file
- Create test method
- Create `test.sh` to verify:
  - Working directory is home
  - Files from host CWD are accessible
  - Can create files that persist to host

### 6. Update Configuration
- Add to `rockerc.yaml` template (commented)

### 7. Run CI and Fix Issues
- Run `pixi run ci`
- Fix any errors
- Iterate until passing

## Technical Considerations
- Need to get home directory path (likely from user extension)
- May need to depend on `user` extension
- Volume mount should be: `host_cwd:container_home`
- WORKDIR should be set to container home directory
