# Base stage: Common utilities for detecting changes
FROM ubuntu:latest AS base

RUN apt-get update && apt-get install -y findutils diffutils && apt-get clean && rm -rf /var/lib/apt/lists/*

# Stage 1: Install curl and detect changes
FROM ubuntu:latest AS curl-stage

RUN find / -type f > /before.txt && \
    apt-get update && apt-get install -y curl && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    find / -type f > /after.txt && \
    diff /before.txt /after.txt | grep ">" | awk '{print $2}' | grep -v '^/proc/' > /curl-changes.txt

# Stage 2: Install git and detect changes
FROM ubuntu:latest AS git-stage

RUN find / -type f > /before.txt && \
    apt-get update && apt-get install -y git && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    find / -type f > /after.txt && \
    diff /before.txt /after.txt | grep ">" | awk '{print $2}' | grep -v '^/proc/' > /git-changes.txt

# Final stage: Combine relevant files
FROM ubuntu:latest AS final

# Copy curl changes
COPY --from=curl-stage /curl-changes.txt /curl-changes.txt
COPY --from=curl-stage / /staging-curl/

# Copy git changes
COPY --from=git-stage /git-changes.txt /git-changes.txt
COPY --from=git-stage / /staging-git/

# Combine only necessary files
RUN mkdir -p /final && \
    xargs -a /curl-changes.txt -I {} cp --parents /staging-curl{} /final/ && \
    xargs -a /git-changes.txt -I {} cp --parents /staging-git{} /final/ && \
    rm -rf /staging-curl /staging-git /curl-changes.txt /git-changes.txt

WORKDIR /final
