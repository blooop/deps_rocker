ENV CONDA_DIR=/opt/miniconda3

RUN cd /tmp && \
    curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh" && \
    bash Miniforge3-$(uname)-$(uname -m).sh -b -p $CONDA_DIR && \
    rm -rf Miniforge3-$(uname)-$(uname -m).sh && \
    ln -s $CONDA_DIR/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo "export PATH=$CONDA_DIR/bin:\$PATH" >> /etc/bash.bashrc
