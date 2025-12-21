# Install git and git-lfs via pixi
RUN --mount=type=cache,target=/root/.cache/pixi,sharing=locked,id=pixi-cache \
    pixi global install git git-lfs
