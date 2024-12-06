# RUN apt-get update && apt-get install -y --no-install-recommends \
#     python3-pip && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip cmake build-essential && apt-get clean && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN pip install isaacsim==4.2.0.2 isaacsim-extscache-kit==4.2.0.2 isaacsim-extscache-kit-sdk==4.2.0.2 isaacsim-extscache-physics==4.2.0.2

ENV OMNI_KIT_ACCEPT_EULA=YES