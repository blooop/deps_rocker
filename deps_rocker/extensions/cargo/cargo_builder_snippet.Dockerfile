# syntax=docker/dockerfile:1.4
ARG CARGO_VERSION=@cargo_version@

FROM @base_image@ AS @builder_stage@

ENV CARGO_HOME=/root/.cargo
ENV RUSTUP_HOME=/root/.rustup

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/tmp/rustup-cache,id=cargo-rustup-cache \
    set -euxo pipefail && \
    mkdir -p /tmp/rustup-cache @builder_output_dir@/root && \
    installer=/tmp/rustup-cache/rustup-init.sh && \
    if [ ! -f "$installer" ]; then \
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o "$installer"; \
    fi && \
    sh "$installer" -y --default-toolchain stable --profile default --no-modify-path && \
    . /root/.cargo/env && \
    mkdir -p @builder_output_dir@ && \
    cp -a /root/.cargo @builder_output_dir@/root/.cargo && \
    cp -a /root/.rustup @builder_output_dir@/root/.rustup && \
    printf 'source /root/.cargo/env\n' > @builder_output_dir@/cargo-env.sh
