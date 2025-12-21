# Install ccache via pixi with cache mount
# ccache is synced at runtime via pixi-global manifest

# Set ccache directory to the mounted volume location
ENV CCACHE_DIR=/root/.ccache
