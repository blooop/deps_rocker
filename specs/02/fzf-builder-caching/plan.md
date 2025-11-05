# Implementation Plan: FZF Builder Stage Caching

## Current Flow
1. Builder stage: Clone fzf repo with `ADD https://github.com/junegunn/fzf.git#master`
2. Builder stage: Copy repo to output directory
3. Main stage: Copy from builder to `/opt/deps_rocker/fzf`
4. User stage: Copy to `~/.fzf` and run `~/.fzf/install --all`
   - **Issue**: Install script downloads binary from GitHub releases here

## Proposed Flow
1. Builder stage: Clone fzf repo using cache mount
2. Builder stage: Download fzf binary using cache mount (if not already cached)
3. Builder stage: Create directory structure with repo + binary
4. Main stage: Copy from builder to `/opt/deps_rocker/fzf`
5. User stage: Copy to `~/.fzf` (binary already present) and run install script
   - Install script detects binary and skips download

## File Changes

### `fzf_builder_snippet.Dockerfile`
```dockerfile
# syntax=docker/dockerfile:1.4
ARG FZF_VERSION=@FZF_VERSION@

@(f"FROM {base_image} AS {builder_stage}")

# Use cache mount for git clone
RUN --mount=type=cache,target=/root/.cache/fzf-repo,id=fzf-repo-cache \
    bash -c "set -euxo pipefail && \
    CACHE_DIR='/root/.cache/fzf-repo' && \
    if [ -d \"\$CACHE_DIR/.git\" ]; then \
        cd \"\$CACHE_DIR\" && \
        git fetch origin master && \
        git reset --hard origin/master; \
    else \
        mkdir -p \"\$CACHE_DIR\" && \
        git clone https://github.com/junegunn/fzf.git \"\$CACHE_DIR\"; \
    fi && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p \"\$OUTPUT_DIR/fzf\" && \
    cp -a \"\$CACHE_DIR/.\" \"\$OUTPUT_DIR/fzf/\""

# Pre-download fzf binary with cache mount
RUN --mount=type=cache,target=/root/.cache/fzf-bins,id=fzf-bins-cache \
    bash -c "set -euxo pipefail && \
    VERSION=\"${FZF_VERSION}\" && \
    ARCH=\$(uname -m) && \
    case \"\$ARCH\" in \
        x86_64) FZF_ARCH='linux_amd64' ;; \
        aarch64) FZF_ARCH='linux_arm64' ;; \
        armv7l) FZF_ARCH='linux_armv7' ;; \
        *) echo \"Unsupported architecture: \$ARCH\"; exit 1 ;; \
    esac && \
    CACHE_FILE=\"/root/.cache/fzf-bins/fzf-\${VERSION}-\${FZF_ARCH}.tar.gz\" && \
    if [ ! -f \"\$CACHE_FILE\" ]; then \
        echo \"Downloading fzf \${VERSION} for \${FZF_ARCH}...\" && \
        curl -fL \"https://github.com/junegunn/fzf/releases/download/v\${VERSION}/fzf-\${VERSION}-\${FZF_ARCH}.tar.gz\" \
            -o \"\$CACHE_FILE\"; \
    else \
        echo \"Using cached fzf binary: \$CACHE_FILE\"; \
    fi && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p \"\$OUTPUT_DIR/fzf/bin\" && \
    tar -xzf \"\$CACHE_FILE\" -C \"\$OUTPUT_DIR/fzf/bin/\""
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
