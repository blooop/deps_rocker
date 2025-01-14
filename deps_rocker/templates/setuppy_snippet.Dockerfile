RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/* 
