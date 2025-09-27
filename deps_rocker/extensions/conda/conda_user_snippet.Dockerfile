

ENV CONDA_DIR=/home/conda

RUN curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"; \
    sudo bash Miniforge3-$(uname)-$(uname -m).sh -b -p $CONDA_DIR; \
    rm -rf Miniforge3-$(uname)-$(uname -m).sh; \
    echo "export PATH=$CONDA_DIR/bin:\$PATH" >> ~/.bashrc ; \
    echo ". $CONDA_DIR/etc/profile.d/conda.sh && conda activate base" >> ~/.bashrc ; \
    sudo chown -R 1000:1000 $CONDA_DIR