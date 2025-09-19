# Install Miniforge into /opt/conda without sudo
ENV CONDA_DIR=/opt/conda
RUN curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh" \
    && bash Miniforge3-$(uname)-$(uname -m).sh -b -p "$CONDA_DIR" \
    && rm -f Miniforge3-$(uname)-$(uname -m).sh

