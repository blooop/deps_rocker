# syntax=docker/dockerfile:1.4
ARG NODE_VERSION=@NODE_VERSION@

@(f"FROM {base_image} AS {builder_stage}")

ENV NVM_DIR=/usr/local/nvm
ENV NODE_VERSION=${NODE_VERSION}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends curl ca-certificates && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/tmp/nvm-install-cache,id=nvm-install-cache \
    bash -c "set -euxo pipefail && \
    mkdir -p /tmp/nvm-install-cache && \
    mkdir -p /opt/deps_rocker/npm && \
    if [ ! -f /tmp/nvm-install-cache/install.sh ]; then \
        curl -sS -o /tmp/nvm-install-cache/install.sh https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.0/install.sh; \
    fi && \
    cp /tmp/nvm-install-cache/install.sh /opt/deps_rocker/npm/install.sh"
