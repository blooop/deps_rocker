# Auto Extension: Search Root Specification

## Problem
The recursive search for project files (e.g., pyproject.toml, package.json, etc.) currently starts at the user's home or workspace root, not the folder that is actually opened in the editor. This leads to incorrect detection and extension activation.

## Requirement
- The recursive search for project files/extensions must start at the folder that is opened in the editor (for rockerc) or the project folder selected by the user (for renv).
- For renv, the search root should be: `~renv/repo_owner/repo_name/branch/repo_name`.
- For rockerc, the search root should be the folder opened directly in the editor.
- All extension detection logic must use this folder as the root for recursive search.

## Acceptance Criteria
- The search root is always the folder opened in the editor (rockerc) or the selected project folder (renv).
- No project files outside this root are considered.
- Extension detection logs show the correct search root.
- Example log: `Scanning workspace: <correct search root>`
