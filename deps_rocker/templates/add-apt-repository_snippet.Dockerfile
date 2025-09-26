# ADD APT REPOSITORIES:: @layer_name
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y \
    software-properties-common

@[for x in data_list]@
RUN add-apt-repository @x
@[end for]@
