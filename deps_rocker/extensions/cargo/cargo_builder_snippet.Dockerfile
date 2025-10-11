# syntax=docker/dockerfile:1.4

@(f"FROM {base_image} AS {builder_stage}")

ENV CARGO_HOME=/root/.cargo
ENV RUSTUP_HOME=/root/.rustup

RUN --mount=type=cache,target=/tmp/rustup-cache,id=cargo-rustup-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /tmp/rustup-cache \"\$OUTPUT_DIR/root\" && \
    installer=/tmp/rustup-cache/rustup-init.sh && \
    if [ ! -f \"\$installer\" ]; then \
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o \"\$installer\"; \
    fi && \
    sh \"\$installer\" -y --default-toolchain stable --profile default --no-modify-path && \
    . /root/.cargo/env && \
    mkdir -p \"\$OUTPUT_DIR\" && \
    cp -a /root/.cargo \"\$OUTPUT_DIR/root/.cargo\" && \
    cp -a /root/.rustup \"\$OUTPUT_DIR/root/.rustup\" && \
    printf 'source /root/.cargo/env\\n' > \"\$OUTPUT_DIR/cargo-env.sh\""
