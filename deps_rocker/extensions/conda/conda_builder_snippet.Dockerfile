# syntax=docker/dockerfile:1.4
ARG CONDA_VERSION=@CONDA_VERSION@

@(f"FROM {base_image} AS {builder_stage}")

ENV CONDA_DIR=/opt/miniconda3

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends bzip2 ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/tmp/miniforge-cache,id=conda-installer-cache \
    bash -c "set -euxo pipefail && \
    OUTPUT_DIR='@(f"{builder_output_dir}")' && \
    mkdir -p /tmp/miniforge-cache \"\$OUTPUT_DIR\" && \
    platform=\"\$(uname)\" && arch=\"\$(uname -m)\" && \
    installer=\"/tmp/miniforge-cache/Miniforge3-\${platform}-\${arch}.sh\" && \
    if [ ! -f \"\$installer\" ]; then \
        curl -sSL \"https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-\${platform}-\${arch}.sh\" -o \"\$installer\"; \
    fi && \
    bash \"\$installer\" -b -p \$CONDA_DIR && \
    \$CONDA_DIR/bin/conda clean -afy && \
    cp -a \$CONDA_DIR \"\$OUTPUT_DIR/miniconda3\" && \
    cp \$CONDA_DIR/etc/profile.d/conda.sh \"\$OUTPUT_DIR/conda.sh\""
