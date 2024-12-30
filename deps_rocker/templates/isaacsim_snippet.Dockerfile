RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip cmake build-essential && apt-get clean && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV OMNI_KIT_ACCEPT_EULA=YES

RUN pip install isaacsim==4.2.0.2 isaacsim-extscache-kit==4.2.0.2 isaacsim-extscache-kit-sdk==4.2.0.2 isaacsim-extscache-physics==4.2.0.2


RUN apt-get update && apt-get install -y --no-install-recommends \
    libxext6 \
    libsm6 \
    libgl1 \
    libglib2.0-0 \
    libxrender1 \
    libglu1-mesa \
    libxi-dev \
    libxmu-dev \
    # libglu1-mesa-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

    # apt_vulkan:
#   - libgtk-3-dev 
#   - libxkbcommon-x11-0 
#   - vulkan-tools 
#   - mesa-vulkan-drivers

# libgtk-3-dev libxkbcommon-x11-0 vulkan-tools mesa-vulkan-drivers

# apt-get install libglu1-mesa libxi-dev libxmu-dev libglu1-mesa-dev