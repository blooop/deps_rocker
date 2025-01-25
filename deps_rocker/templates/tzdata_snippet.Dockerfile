RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    apt-get install -y --no-install-recommends tzdata && \ 
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
