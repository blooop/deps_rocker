RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    git-lfs\
    ca-certificates \
    && apt-get clean && rm -rf /var/lib/apt/lists/* 
