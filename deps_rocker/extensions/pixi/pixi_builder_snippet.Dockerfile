# syntax=docker/dockerfile:1.4
@(f"ARG PIXI_VERSION={PIXI_VERSION}")

@(f"FROM {base_image} AS {builder_stage}")

ARG PIXI_VERSION

# Install pixi from official image (no dependencies needed)
COPY --from=ghcr.io/prefix-dev/pixi:latest /usr/local/bin/pixi /usr/local/bin/pixi

# Create pixi global directory structure and copy pixi binary to it
RUN mkdir -p /root/.pixi/bin && \
    cp /usr/local/bin/pixi /root/.pixi/bin/pixi

ENV PATH="/root/.pixi/bin:$PATH"

# Copy pixi installation to output directory
RUN bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p \"\$OUTPUT_DIR\" && \
    cp -a /root/.pixi \"\$OUTPUT_DIR/.pixi\""
