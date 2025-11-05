# Implementation Plan: FZF Builder Stage Caching

## Current Flow
1. Builder stage: Clone fzf repo with `ADD https://github.com/junegunn/fzf.git#master`
2. Builder stage: Copy repo to output directory
3. Main stage: Copy from builder to `/opt/deps_rocker/fzf`
4. User stage: Copy to `~/.fzf` and run `~/.fzf/install --all`
   - **Issue**: Install script downloads binary from GitHub releases here

## Proposed Flow
1. Builder stage: Clone fzf repo using cache mount
2. Builder stage: Use cache mount for fzf home, run `./install --bin` to download binary
3. Builder stage: Copy repo + binary to output directory
4. Main stage: Copy from builder to `/opt/deps_rocker/fzf`
5. User stage: Copy to `~/.fzf` (binary already present) and run `./install --all`
   - Install script detects binary and only sets up shell integration

## File Changes

### `fzf.py`
- Add `builder_apt_packages = ["git", "curl", "ca-certificates"]`
- This automatically injects apt install into builder stage

### `fzf_builder_snippet.Dockerfile`
- Remove manual apt install (handled by `builder_apt_packages`)
- Add cache mounts for git repo and fzf installation
- Clone/update repo in cache
- Run `./install --bin` to download binary
- Copy complete installation to output directory

### No changes needed for:
- `fzf_snippet.Dockerfile` (main stage copy)
- `fzf_user_snippet.Dockerfile` (runs `./install --all` which detects existing binary and only sets up shell integration)

## Testing
1. First build: Should download binary and cache it
2. Second build: Should reuse cached binary (much faster)
3. Verify `fzf --version` shows correct version in container
4. Test fzf functionality (Ctrl-R, Ctrl-T, etc.)

## Notes
- The fzf install script checks for `~/.fzf/bin/fzf` and skips download if present
- Binary must match the architecture of the final container image
- Cache is keyed by version in filename, so updating FZF_VERSION will trigger new download
