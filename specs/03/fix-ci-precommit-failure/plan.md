# Implementation Plan: CI Pre-commit Dependency

## Analysis
The current setup has two separate workflows:
- `.github/workflows/pre-commit.yaml` - Runs pre-commit on PRs
- `.github/workflows/ci.yml` - Runs main CI pipeline (format, lint, test, coverage)

These run independently in parallel, so pre-commit failures don't block CI execution.

## Solution
Integrate pre-commit as a job dependency in the main CI workflow using GitHub Actions' `needs` keyword.

## Steps

### 1. Update `.github/workflows/ci.yml`
Add a new `pre-commit` job:
```yaml
jobs:
  pre-commit:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v5
      - uses: pre-commit/action@v3.0.1

  ci:
    needs: pre-commit  # Add this dependency
    runs-on: ubuntu-latest
    # ... rest of existing ci job
```

### 2. Keep `.github/workflows/pre-commit.yaml`
- Maintain backwards compatibility
- Some teams may rely on it as a standalone check
- Can be removed in a future cleanup if desired

### 3. Test the Changes
- Run `pixi run ci` locally to ensure nothing breaks
- Push changes and verify GitHub Actions behavior
- Confirm that CI job waits for pre-commit to succeed

## Expected Outcome
- On PR creation/updates, pre-commit runs first
- If pre-commit fails, CI job is skipped with status "skipped"
- If pre-commit passes, CI job runs normally
- Total CI time is reduced when pre-commit fails (saves ~2-10 minutes depending on test suite)

## Risk Assessment
- Low risk: Only changing job dependencies, not workflow logic
- Pre-commit job is identical to existing standalone workflow
- Can be easily reverted if issues arise
