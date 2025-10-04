ENV CONDA_DIR=/opt/miniconda3

# Download and install Miniforge using BuildKit cache for the installer script
RUN --mount=type=cache,target=/tmp/miniforge-cache \
    mkdir -p /tmp/miniforge-cache && \
    platform="$(uname)" && arch="$(uname -m)" && \
    installer="/tmp/miniforge-cache/Miniforge3-${platform}-${arch}.sh" && \
    if [ ! -f "${installer}" ]; then \
        curl -sSL "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-${platform}-${arch}.sh" -o "${installer}"; \
    fi && \
    bash "${installer}" -b -p $CONDA_DIR && \
    ln -s $CONDA_DIR/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo "export PATH=$CONDA_DIR/bin:\$PATH" >> /etc/bash.bashrc
