ENV CONDA_DIR=/opt/miniconda3

# Download and install Miniforge using BuildKit cache for the installer script
RUN --mount=type=cache,target=/tmp/miniforge-cache \
    cd /tmp && \
    curl -L -o /tmp/miniforge-cache/Miniforge3-$(uname)-$(uname -m).sh "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh" && \
    bash /tmp/miniforge-cache/Miniforge3-$(uname)-$(uname -m).sh -b -p $CONDA_DIR && \
    rm -rf /tmp/miniforge-cache/Miniforge3-$(uname)-$(uname -m).sh && \
    ln -s $CONDA_DIR/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo "export PATH=$CONDA_DIR/bin:\$PATH" >> /etc/bash.bashrc
