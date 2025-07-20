from deps_rocker.simple_rocker_extension import SimpleRockerExtension
import subprocess
import os


class Vulkan(SimpleRockerExtension):
    """Install Vulkan SDK and drivers for GPU compute and graphics applications

    This extension installs the Vulkan SDK, development headers, and validation layers.
    It's designed to work alongside the nvidia extension - the nvidia extension provides
    GPU access via --gpus all or --runtime=nvidia, while this extension adds DRI device
    access and installs Vulkan-specific packages.
    """

    name = "vulkan"

    def precondition_environment(self, cliargs):
        """Set up host environment for GPU/X11 access"""
        try:
            # Allow X11 connections from local containers
            subprocess.run(["xhost", "+local:"], check=False, capture_output=True)
            print("Enabled X11 access for local containers")
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("Warning: Could not run xhost - X11 forwarding may not work")

        # Disable Wayland display to force X11 for better compatibility
        if "WAYLAND_DISPLAY" in os.environ:
            print("Note: Disabling Wayland display for container compatibility")

    def invoke_after(self, cliargs) -> set:
        """Vulkan should be loaded after nvidia extension if present

        This ensures that nvidia extension sets up GPU access first,
        then vulkan adds additional DRI device access.
        """
        _ = cliargs  # Suppress unused argument warning
        return {"nvidia"}

    def get_docker_args(self, cliargs) -> str:
        """Add comprehensive GPU device access for Vulkan applications

        Note: This complements the nvidia extension's GPU access. The nvidia extension
        provides --gpus all or --runtime=nvidia for CUDA/OpenGL access, while this
        adds broader device access needed for Vulkan hardware acceleration.
        """
        _ = cliargs  # Suppress unused argument warning
        import os
        import grp

        # Build device arguments only for devices that exist, are character devices, and are accessible
        import stat

        def is_char_device_and_accessible(path):
            try:
                st = os.stat(path)
                # Check if it's a character device
                if not stat.S_ISCHR(st.st_mode):
                    return False
                # Check read and write permissions for the current user
                return os.access(path, os.R_OK | os.W_OK)
            except Exception:
                return False

        devices = []
        for dev_path in ["/dev/dri", "/dev/kfd", "/dev/dxg"]:
            if os.path.exists(dev_path) and is_char_device_and_accessible(dev_path):
                devices.append(f"--device {dev_path}")

        # Build volume mounts only for paths that exist
        volumes = []
        if os.path.exists("/usr/lib/x86_64-linux-gnu/dri"):
            volumes.append("-v /usr/lib/x86_64-linux-gnu/dri:/usr/lib/x86_64-linux-gnu/dri:ro")
        if os.path.exists("/usr/share/libdrm"):
            volumes.append("-v /usr/share/libdrm:/usr/share/libdrm:ro")

        # Add groups only if they exist on the host system
        groups = []
        try:
            grp.getgrnam("video")
            groups.append("--group-add video")
        except KeyError:
            pass
        try:
            grp.getgrnam("render")
            groups.append("--group-add render")
        except KeyError:
            pass

        # Combine all arguments
        args = devices + volumes + groups
        return " ".join(args)
