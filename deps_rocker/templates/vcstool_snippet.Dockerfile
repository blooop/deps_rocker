# INSTALLING APT DEPS: @layer_name
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-vcstool \
    && apt-get clean && rm -rf /var/lib/apt/lists/*