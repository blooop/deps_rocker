# Workdir Extension Implementation Plan

## Overview
Implement a rocker extension that allows users to specify a working directory for their Docker container using the `--workdir` flag.

## Implementation Details

### Extension Structure
1. Create `deps_rocker/extensions/workdir/` directory
2. Implement `Workdir` class inheriting from `SimpleRockerExtension`
3. Override `get_snippet()` to return WORKDIR directive
4. Add argument parsing for `--workdir` parameter

### Docker Implementation
- Generate `WORKDIR <path>` in Dockerfile snippet
- No installation script needed (pure Docker directive)

### Testing
- Add to generic extension tests
- Create test.sh to verify PWD is set correctly in container

### Entry Point
- Add to pyproject.toml entry points

## Technical Considerations
- WORKDIR is a Dockerfile directive that sets the working directory for subsequent RUN, CMD, ENTRYPOINT, COPY, and ADD instructions
- The directory will be created if it doesn't exist
- Multiple WORKDIR instructions can be used (relative paths are relative to previous WORKDIR)
