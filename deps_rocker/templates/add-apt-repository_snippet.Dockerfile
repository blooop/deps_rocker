# ADD APT REPOSITORIES:: @layer_name
@[if buildkit_enabled]@
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y \
    software-properties-common
@[else]@
RUN apt-get update && apt-get install -y \
    software-properties-common && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
@[end if]@

@[for x in data_list]@
RUN add-apt-repository @x
@[end for]@
