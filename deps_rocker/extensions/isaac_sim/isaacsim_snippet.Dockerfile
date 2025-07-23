RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \ 
    cmake \
    build-essential \
    libglib2.0-0 \
    libglu1-mesa \
    libxmu-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV OMNI_KIT_ACCEPT_EULA=YES

RUN pip install isaacsim[all,extscache]==4.5.0
