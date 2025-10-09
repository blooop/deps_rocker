# Install jq JSON processor
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends jq; \
    apt-get clean; \
    rm -rf /var/lib/apt/lists/*
