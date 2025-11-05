# Spec: FZF Builder Stage Caching

## Problem
The fzf extension currently re-downloads the fzf binary from GitHub releases on every build during the user install step (`~/.fzf/install --all`). This wastes time and network bandwidth.

## Solution
Run the native fzf install script in the builder stage to download the binary, using BuildKit cache mounts:
1. Cache the git repository clone
2. Run `./install --bin` in builder stage with cache mount (downloads binary only, no shell setup)
3. Copy repo with pre-downloaded binary to builder output directory
4. In user stage, run `./install --all` which detects existing binary and only sets up shell integration

## Implementation Details
- Use `RUN --mount=type=cache` for both git clone and fzf home directory
- Let fzf's native install script handle architecture detection and downloading
- Builder runs `./install --bin` to populate `bin/fzf`
- User stage runs `./install --all` to set up completion/keybindings (skips download)

## Benefits
- Significantly faster builds (skip ~5-10MB download per build)
- Works offline after first download
- No custom download logic - uses fzf's native installer
- Still respects FZF_VERSION via git checkout
