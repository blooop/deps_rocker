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

### `fzf_builder_snippet.Dockerfile`
```dockerfile
# syntax=docker/dockerfile:1.4
ARG FZF_VERSION=@FZF_VERSION@

@(f"FROM {base_image} AS {builder_stage}")

# Use cache mount for git clone and fzf installation
RUN --mount=type=cache,target=/root/.cache/fzf-repo,id=fzf-repo-cache \
    --mount=type=cache,target=/root/.fzf-install,id=fzf-install-cache \
    bash -c "set -euxo pipefail && \
    # Clone or update repo in cache
    CACHE_DIR='/root/.cache/fzf-repo' && \
    if [ -d \"\$CACHE_DIR/.git\" ]; then \
        cd \"\$CACHE_DIR\" && \
        git fetch origin master && \
        git reset --hard origin/master; \
    else \
        mkdir -p \"\$CACHE_DIR\" && \
        git clone https://github.com/junegunn/fzf.git \"\$CACHE_DIR\"; \
    fi && \
    # Copy to fzf install location and run installer with --bin flag
    mkdir -p /root/.fzf-install && \
    rm -rf /root/.fzf-install/* && \
    cp -a \"\$CACHE_DIR/.\" /root/.fzf-install/ && \
    cd /root/.fzf-install && \
    ./install --bin && \
    # Copy completed installation to output directory
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p \"\$OUTPUT_DIR/fzf\" && \
    cp -a /root/.fzf-install/. \"\$OUTPUT_DIR/fzf/\""
```

### No changes needed for:
- `fzf_snippet.Dockerfile` (main stage copy)
- `fzf_user_snippet.Dockerfile` (install script will detect binary)
- `fzf.py` (no logic changes)

## Testing
1. First build: Should download binary and cache it
2. Second build: Should reuse cached binary (much faster)
3. Verify `fzf --version` shows correct version in container
4. Test fzf functionality (Ctrl-R, Ctrl-T, etc.)

## Notes
- The fzf install script checks for `~/.fzf/bin/fzf` and skips download if present
- Binary must match the architecture of the final container image
- Cache is keyed by version in filename, so updating FZF_VERSION will trigger new download
