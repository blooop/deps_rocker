# User-specific Vulkan configuration
# Add Vulkan tools to PATH for easy access
RUN echo 'export PATH=$PATH:/usr/bin' >> $HOME/.bashrc

# Create directory for Vulkan validation layer settings
RUN mkdir -p $HOME/.local/share/vulkan/settings.d

# Set default Vulkan debug settings for development
RUN echo 'export VK_LOADER_DEBUG=all' >> $HOME/.bashrc
RUN echo 'export VK_LAYER_ENABLES=VK_VALIDATION_FEATURE_ENABLE_DEBUG_PRINTF_EXT' >> $HOME/.bashrc
