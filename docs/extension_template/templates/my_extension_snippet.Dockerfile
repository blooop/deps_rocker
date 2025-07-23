# Install whatever your extension needs
RUN apt-get update && apt-get install -y --no-install-recommends \
    my-package \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
