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
1. Add a `pre-commit` job to `.github/workflows/ci.yml`
2. Add `needs: pre-commit` to the existing `ci` job to create dependency
3. Remove the separate `.github/workflows/pre-commit.yaml` to avoid running pre-commit twice

## Status
✅ Implemented - CI workflow updated with pre-commit job dependency, duplicate workflow removed

## Benefits
- Faster feedback on pre-commit failures
- Saves CI resources by not running expensive tests when code style is wrong
- Clear failure ordering in GitHub Actions UI
