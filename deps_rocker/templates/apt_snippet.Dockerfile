# INSTALLING APT DEPS: @layer_name
@[if buildkit_enabled]@
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
@[if data_list]@
@( '\n'.join(['    ' + pkg + ' \\' for pkg in data_list[:-1]] + ['    ' + data_list[-1]]) )
@[end if]@
@[else]@
RUN apt-get update && apt-get install -y --no-install-recommends \
@[if data_list]@
@( '\n'.join(['    ' + pkg + ' \\' for pkg in data_list[:-1]] + ['    ' + data_list[-1] + ' \\']) )
@[end if]@
    && apt-get clean && rm -rf /var/lib/apt/lists/*
@[end if]@
