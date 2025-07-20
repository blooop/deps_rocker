from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Vulkan(SimpleRockerExtension):
    """Install Vulkan SDK and drivers for GPU compute and graphics applications
    
    This extension installs the Vulkan SDK, development headers, and validation layers.
    It's designed to work alongside the nvidia extension - the nvidia extension provides
    GPU access via --gpus all or --runtime=nvidia, while this extension adds DRI device
    access and installs Vulkan-specific packages.
    """

    name = "vulkan"

    def invoke_after(self, cliargs) -> set:
        """Vulkan should be loaded after nvidia extension if present
        
        This ensures that nvidia extension sets up GPU access first,
        then vulkan adds additional DRI device access.
        """
        _ = cliargs  # Suppress unused argument warning
        return {"nvidia"}

    def get_docker_args(self, cliargs) -> str:
        """Add DRI device access for Vulkan applications
        
        Note: This complements the nvidia extension's GPU access. The nvidia extension
        provides --gpus all or --runtime=nvidia for CUDA/OpenGL access, while this
        adds --device /dev/dri for direct rendering interface access needed by some
        Vulkan applications.
        """
        _ = cliargs  # Suppress unused argument warning
        return "--device /dev/dri"
