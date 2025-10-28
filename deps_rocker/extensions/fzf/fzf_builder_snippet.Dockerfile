# syntax=docker/dockerfile:1.4
ARG FZF_VERSION=@FZF_VERSION@

@(f"FROM {base_image} AS {builder_stage}")

ADD https://github.com/junegunn/fzf.git#master /tmp/fzf

RUN bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p \"\$OUTPUT_DIR\" && \
    cp -a /tmp/fzf \"\$OUTPUT_DIR/fzf\""
