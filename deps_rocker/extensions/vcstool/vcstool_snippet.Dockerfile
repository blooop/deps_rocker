RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    (apt-get install -y --no-install-recommends python3-vcstool || \
     (apt-get install -y --no-install-recommends python3-pip && \
      (pip install --break-system-packages vcstool 2>/dev/null || pip install vcstool))) && \
    rm -rf /var/lib/apt/lists/*

#loops through all the *.repos files that were found and imports them with the same folder structure
@[for dep in depend_repos]@
COPY @dep["dep"] /dependencies/@dep["dep"]
RUN mkdir -p ./dependencies/@dep["path"] ; vcs import --recursive ./dependencies/@dep["path"] < /dependencies/@dep["dep"]
# RUN vcs import . < /dependencies/@dep["dep"]
@[end for]@
