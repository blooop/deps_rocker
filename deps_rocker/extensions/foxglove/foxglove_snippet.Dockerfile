# Install Foxglove Studio via snap
RUN apt-get update && apt-get install --no-install-recommends -y \
    snapd \
    && rm -rf /var/lib/apt/lists/*

# Install Foxglove Studio using snap
RUN snap install foxglove-studio