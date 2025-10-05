# syntax=docker/dockerfile:1.4
ARG PIXI_VERSION=@pixi_version@

FROM @base_image@ AS @builder_stage@

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/pixi-install-cache,id=pixi-install-cache \
    set -euxo pipefail; \
    mkdir -p /root/.cache/pixi-install-cache @builder_output_dir@; \
    script=/root/.cache/pixi-install-cache/install.sh; \
    if [ ! -f "$script" ]; then \
        curl -fsSL https://pixi.sh/install.sh -o "$script"; \
    fi; \
    bash "$script"; \
    cp -a /root/.pixi @builder_output_dir@/.pixi
