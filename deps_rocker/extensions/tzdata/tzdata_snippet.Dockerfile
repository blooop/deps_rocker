RUN set -eux; \
    export DEBIAN_FRONTEND=noninteractive; \
    export TZ=Etc/UTC; \
    apt-get update; \
    apt-get install -y --no-install-recommends tzdata; \
    ln -fs "/usr/share/zoneinfo/${TZ}" /etc/localtime; \
    echo "${TZ}" > /etc/timezone; \
    dpkg-reconfigure -f noninteractive tzdata; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/*
