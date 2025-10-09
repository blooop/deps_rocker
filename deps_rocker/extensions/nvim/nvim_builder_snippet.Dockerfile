# syntax=docker/dockerfile:1.4
@(f"FROM {base_image} AS {builder_stage}")
ARG NVIM_VERSION=v0.11.4

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/nvim-downloads,id=nvim-downloads \
    export NVIM_VERSION=${NVIM_VERSION} && \
    bash -c 'set -euxo pipefail && \
    OUTPUT_DIR="@(f"{builder_output_dir}")" && \
    mkdir -p /root/.cache/neovim-downloads "$OUTPUT_DIR" && \
    NVIM_ARCHIVE="/root/.cache/neovim-downloads/nvim-${NVIM_VERSION}-linux-x86_64.tar.gz" && \
    if [ ! -f "$NVIM_ARCHIVE" ]; then \
        curl -fsSL "https://github.com/neovim/neovim/releases/download/${NVIM_VERSION}/nvim-linux-x86_64.tar.gz" \
             -o "$NVIM_ARCHIVE"; \
    fi && \
    tar -xzf "$NVIM_ARCHIVE" -C /tmp && \
    cp -a /tmp/nvim-linux-x86_64 "$OUTPUT_DIR/nvim"'
