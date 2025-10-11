# syntax=docker/dockerfile:1.4
ARG NODE_VERSION=@NODE_VERSION@

@(f"FROM {base_image} AS {builder_stage}")

ARG NODE_VERSION
ENV NVM_DIR=/usr/local/nvm

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends curl ca-certificates git && \
    rm -rf /var/lib/apt/lists/*

# Clone nvm from git with caching
RUN --mount=type=cache,target=/tmp/nvm-git-cache,id=nvm-git-cache \
    bash -c "set -e && \
    if [ -d /tmp/nvm-git-cache/.git ]; then \
        cd /tmp/nvm-git-cache && git fetch --tags && git checkout v0.40.0; \
    else \
        git clone https://github.com/nvm-sh/nvm.git /tmp/nvm-git-cache && \
        cd /tmp/nvm-git-cache && git checkout v0.40.0; \
    fi && \
    cp -r /tmp/nvm-git-cache /usr/local/nvm"

# Install Node.js using nvm with cached downloads - use template substitution for version
RUN --mount=type=cache,target=/usr/local/nvm/.cache,id=nvm-node-cache \
    bash -c "set -e && \
    . \$NVM_DIR/nvm.sh && \
    nvm install @(NODE_VERSION)"
