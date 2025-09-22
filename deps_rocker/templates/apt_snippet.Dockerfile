# INSTALLING APT DEPS: @layer_name
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    @[for x in data_list]@
    @x \
    @[end for]@
