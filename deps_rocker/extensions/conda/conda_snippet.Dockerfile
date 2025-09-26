RUN curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh && \
    /opt/conda/bin/conda clean -ay

ENV PATH="/opt/conda/bin:$PATH"

RUN conda config --set always_yes yes --set changeps1 no && \
    conda update -q conda && \
    conda init bash