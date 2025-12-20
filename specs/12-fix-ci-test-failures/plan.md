# Implementation Plan: Fix CI Test Failures

## Investigation Steps

1. **Understand test failures**
   - Both tests build and run Docker containers with extensions
   - Tests verify output contains specific strings
   - CI environment runs inside devcontainer with different user context

2. **Identify root causes**
   - Check if HOME directory assumptions are wrong
   - Check if user context is different (vscode vs root)
   - Check if Docker BuildKit behaves differently in CI
   - Check if there are missing dependencies in CI environment

3. **Fix approaches**
   - Make tests more robust to different user contexts
   - Fix HOME directory assumptions
   - Ensure test scripts handle different environments
   - Add better error reporting to understand CI failures

## Implementation

1. Run tests locally to understand baseline behavior
2. Add debugging output to understand what's happening in CI
3. Fix identified issues
4. Verify fixes with `pixi run ci`
