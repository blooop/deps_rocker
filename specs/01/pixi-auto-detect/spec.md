# Pixi Auto-Detect Extension Spec

## Goal
Update the pixi extension's auto-detection logic so that it can:
- Detect a filename (e.g., `pyproject.toml`) with an optional content search.
- For `pyproject.toml`, search for a `[tool.pixi]` section to auto-activate the extension.

## Requirements
- Auto-detection should support both filename and content search.
- Pixi extension should only activate if `[tool.pixi]` is present in `pyproject.toml`.
- Should be robust to whitespace and comments.

## References
- https://github.com/prefix-dev/pixi/
- Error: found pyproject.toml without tool.pixi section at directory /grip

## Acceptance Criteria
- Pixi extension only activates if `[tool.pixi]` is found in `pyproject.toml`.
- Auto-detection logic supports content search for any file.
