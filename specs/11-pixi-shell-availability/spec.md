# Spec 11: Pixi Shell Availability

## Problem
When entering a container, pixi is not available in the shell by default. Users must manually source pixi's initialization script to use it.

## Solution
Ensure pixi is automatically available in all shells by adding its initialization to system-wide shell configuration files (`/etc/bash.bashrc` and `/etc/profile.d/pixi.sh`).

## Implementation
- Modify the pixi extension's Dockerfile snippet to add pixi initialization to `/etc/bash.bashrc` for interactive shells
- Add `/etc/profile.d/pixi.sh` for login shells
- Ensure pixi binary path is in the system PATH

## Success Criteria
- Running `pixi` command works immediately after entering a container without manual sourcing
- Tests pass with pixi available by default
