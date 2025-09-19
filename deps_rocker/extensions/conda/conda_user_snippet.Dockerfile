# Add conda to PATH and activate base on login
ENV CONDA_DIR=/opt/conda
RUN echo "export PATH=$CONDA_DIR/bin:\$PATH" >> ~/.bashrc \
 && echo ". $CONDA_DIR/etc/profile.d/conda.sh && conda activate base" >> ~/.bashrc

