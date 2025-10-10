# Auto Extension Specification

## Overview
Create an `auto` extension that automatically detects and depends on other extensions based on files present in the workspace.

## Behavior
The extension scans the current working directory and dynamically adds dependencies based on detected file patterns:

- `pixi.toml` → pixi
- `pyproject.toml`, `requirements*.txt`, `.python-version`, `poetry.lock` → uv
- `package.json` → npm
- `Cargo.toml` → cargo
- `environment.yml`, `environment.yaml` → conda
- `package.xml` → ros_jazzy
- `*.cpp`, `*.hpp`, `*.cc`, `*.cxx`, `*.h`, `*.c` → ccache

## Implementation Requirements
1. Override `required()` method to return dependencies based on file detection
2. Use centralized `get_workspace_path()` function for consistent directory handling
3. Scan workspace for matching files/patterns
4. Return appropriate extension dependencies as a set
5. Should not require any Dockerfile snippet (no installation needed)
6. Works as a convenience wrapper to auto-enable relevant extensions

## Workspace Path Centralization
A `get_workspace_path()` utility function was added to `simple_rocker_extension.py` to centralize workspace directory access across all extensions. This ensures consistent behavior when determining which directory to scan for project files.
