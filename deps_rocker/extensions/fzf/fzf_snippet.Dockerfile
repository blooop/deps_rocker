# syntax=docker/dockerfile:1.4

# Install fzf via pixi with cache mount
RUN --mount=type=cache,target=/root/.cache/pixi,id=pixi-cache \
    pixi global install fzf
