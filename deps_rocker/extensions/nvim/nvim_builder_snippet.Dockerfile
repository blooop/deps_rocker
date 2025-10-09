# syntax=docker/dockerfile:1.4
@(f"FROM {base_image} AS {builder_stage}")
ARG nvim_VERSION=v0.11.4

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/nvim-downloads,id=nvim-downloads \
    export NVIM_VERSION=${nvim_VERSION} && \
    bash -c 'set -euxo pipefail && \
    OUTPUT_DIR="@(f"{builder_output_dir}")" && \
    mkdir -p /root/.cache/nvim-downloads "$OUTPUT_DIR" && \
    NVIM_ARCHIVE="/root/.cache/nvim-downloads/nvim-${NVIM_VERSION}-linux-x86_64.tar.gz" && \
    if [ ! -f "$NVIM_ARCHIVE" ]; then \
        curl -fsSL "https://github.com/nvim/nvim/releases/download/${NVIM_VERSION}/nvim-linux-x86_64.tar.gz" \
             -o "$NVIM_ARCHIVE"; \
    fi && \
    tar -xzf "$NVIM_ARCHIVE" -C /tmp && \
    cp -a /tmp/nvim-linux-x86_64 "$OUTPUT_DIR/nvim"'
