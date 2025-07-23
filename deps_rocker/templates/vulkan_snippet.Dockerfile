# Ensure the render group exists (GID 107 is standard on Ubuntu)
RUN groupadd -g 107 render || true
# Install Vulkan SDK and drivers
# This extension works alongside the nvidia extension for complete GPU support:
# - nvidia extension provides GPU access via --gpus all or --runtime=nvidia
# - vulkan extension adds DRI device access and Vulkan-specific packages
ENV DEBIAN_FRONTEND=noninteractive


# Add LunarG Vulkan SDK repository for Ubuntu 22.04 (jammy) and 24.04 (noble)
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    gnupg2 \
    software-properties-common \
    && wget -qO /etc/apt/trusted.gpg.d/lunarg.asc https://packages.lunarg.com/lunarg-signing-key-pub.asc \
    && . /etc/os-release \
    && if [ "$VERSION_CODENAME" = "noble" ]; then \
        wget -qO /etc/apt/sources.list.d/lunarg-vulkan-noble.list https://packages.lunarg.com/vulkan/lunarg-vulkan-noble.list; \
    else \
        wget -qO /etc/apt/sources.list.d/lunarg-vulkan-jammy.list https://packages.lunarg.com/vulkan/lunarg-vulkan-jammy.list; \
    fi \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

# Install Vulkan SDK and related packages
RUN . /etc/os-release \
    && if [ "$VERSION_CODENAME" = "noble" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            vulkan-sdk \
            vulkan-tools \
            vulkan-validationlayers \
            vulkan-utility-libraries-dev \
            mesa-vulkan-drivers \
            mesa-utils \
            vainfo \
            intel-media-va-driver \
            libassimp-dev \
            libglfw3-dev \
            libxinerama-dev \
            libxcursor-dev \
            libxi-dev \
            && apt-get clean \
            && rm -rf /var/lib/apt/lists/*; \
    else \
        apt-get update && apt-get install -y --no-install-recommends \
            vulkan-sdk \
            vulkan-tools \
            vulkan-validationlayers \
            vulkan-validationlayers-dev \
            mesa-vulkan-drivers \
            mesa-utils \
            vainfo \
            intel-media-va-driver \
            libassimp-dev \
            libglfw3-dev \
            libxinerama-dev \
            libxcursor-dev \
            libxi-dev \
            && apt-get clean \
            && rm -rf /var/lib/apt/lists/*; \
    fi


# Set up XDG_RUNTIME_DIR for graphics apps
ENV XDG_RUNTIME_DIR=/tmp/xdg
RUN mkdir -p /tmp/xdg && chmod 700 /tmp/xdg

# Set Vulkan environment variables
ENV VK_LAYER_PATH=/usr/share/vulkan/explicit_layer.d

# Dynamically set VK_ICD_FILENAMES for available drivers
RUN bash -c 'VK_ICD=""; for f in /usr/share/vulkan/icd.d/*.json; do VK_ICD="$VK_ICD${VK_ICD:+:}$f"; done; echo "export VK_ICD_FILENAMES=$VK_ICD" >> /etc/profile.d/vulkan_icd.sh'
