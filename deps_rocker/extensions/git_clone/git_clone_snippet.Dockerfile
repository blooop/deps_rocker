# Install git and git-lfs via pixi
RUN --mount=type=cache,target=/root/.cache/pixi,id=pixi-cache \
    pixi global install git git-lfs