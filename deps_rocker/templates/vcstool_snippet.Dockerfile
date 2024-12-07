RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    git \
    git-lfs \
    && apt-get clean && rm -rf /var/lib/apt/lists/* 

RUN pip install vcstool

WORKDIR /workspaces

#loops through all the *.repos files that were found and imports them with the same folder structure
@[for dep in depend_repos]@
COPY @dep["dep"] /dependencies/@dep["dep"]
RUN mkdir -p ./dependencies/@dep["path"] ; vcs import ./dependencies/@dep["path"] < /dependencies/@dep["dep"]
@[end for]@