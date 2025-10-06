# syntax=docker/dockerfile:1.4

@(f"FROM {base_image} AS {builder_stage}")

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends curl ca-certificates && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/tmp/claude-install-cache \
    bash -c "set -euxo pipefail && \
    mkdir -p /tmp/claude-install-cache && \
    mkdir -p @(builder_output_dir) && \
    if [ ! -f /tmp/claude-install-cache/bootstrap.sh ]; then \
        curl -sSL -o /tmp/claude-install-cache/bootstrap.sh https://claude.ai/install.sh; \
    fi && \
    cp /tmp/claude-install-cache/bootstrap.sh @(builder_output_dir)/install.sh"

COPY claude-wrapper.sh @(builder_output_dir)/claude-wrapper.sh
