# syntax=docker/dockerfile:1.4
RUN --mount=type=cache,target=/root/.cache/pixi,sharing=locked,id=pixi-cache \
    pixi global install --channel https://prefix.dev/github-releases neovim
