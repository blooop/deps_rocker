# Spec: Optimize Pixi Permission Setting

## Problem
The pixi extension build has a slow step (`RUN chmod -R a+rx /opt/pixi`) that walks tens of thousands of files in the pixi environment, defeating the performance goal of using pixi.

## Root Cause
Docker's overlay filesystem makes recursive metadata operations (chmod -R) extremely slow. The current implementation:
1. Copies files from builder stage
2. Then runs separate `chmod -R` command that walks entire tree

## Solution
Use `COPY --chmod=755` flag to set directory/executable permissions during copy operation, eliminating the separate chmod step entirely.

## Requirements
- Remove `RUN chmod -R a+rx @(pixi_runtime_root)` from pixi_snippet.Dockerfile
- Add `--chmod=755` to COPY commands that copy directories
- Maintain rocker compatibility (any UID must be able to read/execute)
- Verify permissions work for non-root users

## Expected Outcome
Significantly faster builds by eliminating the recursive chmod walk while maintaining proper permissions for rocker's user mapping.
