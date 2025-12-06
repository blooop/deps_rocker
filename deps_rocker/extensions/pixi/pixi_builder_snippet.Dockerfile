# syntax=docker/dockerfile:1.4
@(f"ARG PIXI_VERSION={PIXI_VERSION}")

@(f"FROM {base_image} AS {builder_stage}")

ARG PIXI_VERSION

RUN bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p \"\$OUTPUT_DIR\" && \
    cp -a /root/.pixi \"\$OUTPUT_DIR/.pixi\""
