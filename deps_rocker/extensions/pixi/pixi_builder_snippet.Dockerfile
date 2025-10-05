# syntax=docker/dockerfile:1.4
@(f"ARG PIXI_VERSION={PIXI_VERSION}")

@(f"FROM {base_image} AS {builder_stage}")

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/pixi-install-cache,id=pixi-install-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /root/.cache/pixi-install-cache \"\$OUTPUT_DIR\" && \
    script=/root/.cache/pixi-install-cache/install.sh && \
    if [ ! -f \"\$script\" ]; then \
        curl -fsSL https://pixi.sh/install.sh -o \"\$script\"; \
    fi && \
    bash \"\$script\" && \
    cp -a /root/.pixi \"\$OUTPUT_DIR/.pixi\""
