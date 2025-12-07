# Install ccache via pixi with cache mount
RUN --mount=type=cache,target=/root/.cache/pixi,id=pixi-cache \
    pixi global install ccache

# Set ccache directory to the mounted volume location
ENV CCACHE_DIR=/root/.ccache
