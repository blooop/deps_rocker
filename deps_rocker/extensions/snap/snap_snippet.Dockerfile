# Install snapd package manager
RUN apt-get update && apt-get install --no-install-recommends -y \
    snapd \
    && rm -rf /var/lib/apt/lists/*