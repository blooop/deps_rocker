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
2. Scan workspace for matching files/patterns
3. Return appropriate extension dependencies as a set
4. Should not require any Dockerfile snippet (no installation needed)
5. Works as a convenience wrapper to auto-enable relevant extensions
