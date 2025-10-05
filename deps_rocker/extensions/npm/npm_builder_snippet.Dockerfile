# syntax=docker/dockerfile:1.4
ARG NODE_VERSION=@node_version@
ARG NPM_VERSION=@npm_version@
ARG NVM_VERSION=@nvm_version@

FROM @base_image@ AS @builder_stage@

ENV NVM_DIR=/usr/local/nvm

RUN mkdir -p $NVM_DIR

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends bash build-essential ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/tmp/nvm-install-cache,id=npm-nvm-cache \
    --mount=type=cache,target=/root/.cache/node-gyp,id=npm-node-gyp-cache \
    bash -lc 'set -euxo pipefail; \
        mkdir -p /tmp/nvm-install-cache @builder_output_dir@; \
        script=/tmp/nvm-install-cache/install.sh; \
        if [ ! -f "$script" ]; then \
            curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.0/install.sh -o "$script"; \
        fi; \
        export NVM_DIR=$NVM_DIR; \
        bash "$script"; \
        source "$NVM_DIR/nvm.sh"; \
        nvm install $NODE_VERSION; \
        nvm use $NODE_VERSION; \
        nvm alias default $NODE_VERSION; \
        npm install -g npm@@$NPM_VERSION; \
        cp -a "$NVM_DIR" @builder_output_dir@/nvm; \
        printf "export NVM_DIR=%s\\n" "$NVM_DIR" > @builder_output_dir@/nvm-env.sh; \
        printf "[ -s \"\\$NVM_DIR/nvm.sh\\" ] && . \"\\$NVM_DIR/nvm.sh\\"\\n" >> @builder_output_dir@/nvm-env.sh; \
        printf "export PATH=\\"\\$NVM_DIR/versions/node/v$NODE_VERSION/bin:\\$PATH\\"\\n" >> @builder_output_dir@/nvm-env.sh; \
        npm --version'
