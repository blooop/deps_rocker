# Install Vulkan SDK and drivers
# This extension works alongside the nvidia extension for complete GPU support:
# - nvidia extension provides GPU access via --gpus all or --runtime=nvidia
# - vulkan extension adds DRI device access and Vulkan-specific packages
ENV DEBIAN_FRONTEND=noninteractive

# Add LunarG Vulkan SDK repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    gnupg2 \
    software-properties-common \
    && wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | apt-key add - \
    && wget -qO /etc/apt/sources.list.d/lunarg-vulkan-jammy.list \
        https://packages.lunarg.com/vulkan/lunarg-vulkan-jammy.list \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

# Install Vulkan SDK and related packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    vulkan-sdk \
    vulkan-tools \
    vulkan-validationlayers \
    vulkan-validationlayers-dev \
    mesa-vulkan-drivers \
    libassimp-dev \
    libglfw3-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set Vulkan environment variables
ENV VK_LAYER_PATH=/usr/share/vulkan/explicit_layer.d
ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/intel_icd.x86_64.json:/usr/share/vulkan/icd.d/radeon_icd.x86_64.json:/usr/share/vulkan/icd.d/nvidia_icd.json
