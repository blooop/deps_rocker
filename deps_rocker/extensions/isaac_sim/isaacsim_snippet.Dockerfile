RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    cmake \
    build-essential \
    libglib2.0-0 \
    libglu1-mesa \
    libxmu-dev

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV OMNI_KIT_ACCEPT_EULA=YES

RUN pip install isaacsim[all,extscache]==4.5.0
