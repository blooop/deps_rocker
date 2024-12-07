RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    git \
    git-lfs \
    && apt-get clean && rm -rf /var/lib/apt/lists/* 

RUN pip install vcstool

