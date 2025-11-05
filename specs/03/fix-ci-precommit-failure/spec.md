# CI Pre-commit Dependency Specification

## Overview
Update the GitHub Actions CI workflow to ensure that if the pre-commit step fails, the rest of the CI steps don't run.

## Current Behavior
- `pre-commit.yaml` and `ci.yml` run as separate, independent workflows on pull requests
- Pre-commit failures don't prevent the CI job from running
- Both workflows run in parallel, wasting CI resources when pre-commit fails

## Desired Behavior
- Pre-commit checks run first as a separate job in the CI workflow
- Main CI job depends on pre-commit job success
- If pre-commit fails, the main CI job is skipped
- Reduces wasted CI time and makes failures more obvious

## Implementation
Modify `.github/workflows/ci.yml` to:
1. Add a `pre-commit` job that runs pre-commit checks
2. Add `needs: pre-commit` to the existing `ci` job to create dependency
3. Keep the separate `pre-commit.yaml` workflow for backwards compatibility

## Status
✅ Implemented - CI workflow updated with pre-commit job dependency

## Benefits
- Faster feedback on pre-commit failures
- Saves CI resources by not running expensive tests when code style is wrong
- Clear failure ordering in GitHub Actions UI
