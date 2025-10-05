# syntax=docker/dockerfile:1.4
ARG CONDA_VERSION=@conda_version@

FROM @base_image@ AS @builder_stage@

ENV CONDA_DIR=/opt/miniconda3

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends bzip2 ca-certificates curl && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/tmp/miniforge-cache,id=conda-installer-cache \
    set -euxo pipefail; \
    mkdir -p /tmp/miniforge-cache @builder_output_dir@; \
    platform="$$(uname)"; arch="$$(uname -m)"; \
    installer="/tmp/miniforge-cache/Miniforge3-$${platform}-$${arch}.sh"; \
    if [ ! -f "$$installer" ]; then \
        curl -sSL "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$${platform}-$${arch}.sh" -o "$$installer"; \
    fi; \
    bash "$$installer" -b -p $$CONDA_DIR; \
    $$CONDA_DIR/bin/conda clean -afy; \
    cp -a $$CONDA_DIR @builder_output_dir@/miniconda3; \
    cp $$CONDA_DIR/etc/profile.d/conda.sh @builder_output_dir@/conda.sh
