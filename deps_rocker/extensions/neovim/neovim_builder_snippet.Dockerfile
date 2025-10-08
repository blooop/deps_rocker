# syntax=docker/dockerfile:1.4
ARG NEOVIM_VERSION=v0.11.4

@(f"FROM {base_image} AS {builder_stage}")
ARG NEOVIM_VERSION

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/neovim-downloads,id=nvim-downloads \
    export NVIM_VERSION="${NEOVIM_VERSION}" && \
    bash -c 'set -euxo pipefail && \
    : "${NVIM_VERSION:?NEOVIM_VERSION must be set}"; \
    OUTPUT_DIR="@(f"{builder_output_dir}")" && \
    CACHE_DIR="/root/.cache/neovim-downloads" && \
    mkdir -p "$CACHE_DIR" "$OUTPUT_DIR" && \
    NVIM_ARCHIVE="$CACHE_DIR/nvim-${NVIM_VERSION}-linux64.tar.gz" && \
    if [ ! -f "$NVIM_ARCHIVE" ]; then \
        curl -fsSL "https://github.com/neovim/neovim/releases/download/${NVIM_VERSION}/nvim-linux64.tar.gz" \
             -o "$NVIM_ARCHIVE"; \
    fi && \
    tar -xzf "$NVIM_ARCHIVE" -C /tmp && \
    EXTRACTED_DIR=$(find /tmp -maxdepth 1 -type d \( -name nvim-linux64 -o -name nvim-linux-x86_64 \) -print -quit) && \
    if [ -z "$EXTRACTED_DIR" ]; then \
        echo "Failed to locate extracted Neovim directory" >&2; \
        exit 1; \
    fi && \
    rm -rf "$OUTPUT_DIR/nvim" && \
    cp -a "$EXTRACTED_DIR" "$OUTPUT_DIR/nvim" && \
    rm -rf "$EXTRACTED_DIR"'
