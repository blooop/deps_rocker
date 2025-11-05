# Spec: FZF Builder Stage Caching

## Problem
The fzf extension currently re-downloads the fzf binary from GitHub releases on every build during the user install step (`~/.fzf/install --all`), even though the builder stage already clones the repository. This wastes time and network bandwidth.

## Solution
Move the binary download to the builder stage using BuildKit cache mounts:
1. Cache the git repository clone
2. Pre-download the versioned fzf binary (matching `FZF_VERSION`) in the builder stage
3. Copy both repo and binary to builder output directory
4. In user stage, the install script will find the pre-downloaded binary and skip the download

## Implementation Details
- Use `RUN --mount=type=cache,target=/root/.cache/fzf` for binary downloads
- Download architecture-appropriate binary based on `uname -m` and `FZF_VERSION`
- Keep binaries in cache with versioned filenames (e.g., `fzf-${VERSION}-${ARCH}.tar.gz`)
- Install script will detect existing `~/.fzf/bin/fzf` binary and skip download

## Benefits
- Significantly faster builds (skip ~5-10MB download per build)
- Works offline after first download
- Still respects FZF_VERSION for updates
