# syntax=docker/dockerfile:1.4
ARG MINIFORGE_VERSION=@MINIFORGE_VERSION@
ARG CONDA_VERSION=@CONDA_VERSION@

ENV CONDA_DIR=/opt/miniconda3

# Copy Miniforge installation prepared in builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/miniconda3 $CONDA_DIR")
@(f"COPY --from={builder_stage} {builder_output_dir}/conda.sh /etc/profile.d/conda.sh")
RUN chmod 644 /etc/profile.d/conda.sh && \
    echo 'export PATH=$CONDA_DIR/bin:$PATH' >> /etc/bash.bashrc && \
    echo 'source /etc/profile.d/conda.sh' >> /etc/bash.bashrc && \
    echo 'source /etc/profile.d/conda.sh' >> /root/.bashrc
ENV PATH="$CONDA_DIR/bin:${PATH}"
