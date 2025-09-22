# Install pixi package manager
RUN curl -fsSL https://pixi.sh/install.sh | bash

# Set up pixi cache and install directories
RUN mkdir -p /root/.pixi/cache && chmod 755 /root/.pixi/cache
RUN mkdir -p /root/.cache/pixi && chmod 755 /root/.cache/pixi