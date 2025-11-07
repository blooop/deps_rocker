import os
import pkgutil
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Foxglove(SimpleRockerExtension):
    """Install Foxglove Studio for robotics data visualization"""

    name = "foxglove"
    depends_on_extension = ("curl", "x11")
    builder_apt_packages = ["curl", "ca-certificates"]
    empy_args = {"FOXGLOVE_VERSION": "2.39.1"}
    apt_packages = [
        "libgtk-3-0",
        "libnotify4",
        "libnss3",
        "libxtst6",
        "xdg-utils",
        "libatspi2.0-0",
        "libdrm2",
        "libgbm1",
        "libxcb-dri3-0",
        "libasound2t64",
        "desktop-file-utils",
        "gnupg",
    ]

    def get_docker_args(self, cliargs) -> str:
        """
        Mount Foxglove Agent persistent storage:
        - Named volume for agent index: foxglove-agent-index:/index
        - Host directory for recordings: ${HOME}/foxglove_recordings:/storage

        Note: Browser integration is provided via x11 dependency for GUI forwarding.
        """
        home_dir = os.path.expanduser("~")
        recordings_dir = os.path.join(home_dir, "foxglove_recordings")

        # Create recordings directory if it doesn't exist
        os.makedirs(recordings_dir, exist_ok=True)

        return f' -v foxglove-agent-index:/index -v "{recordings_dir}:/storage"'

    def get_files(self, cliargs) -> dict[str, str]:
        files = super().get_files(cliargs) or {}
        wrapper_data = pkgutil.get_data(__name__, "foxglove_wrapper.sh")
        if wrapper_data is None:
            raise FileNotFoundError("foxglove_wrapper.sh not found in package data")
        files["foxglove_wrapper.sh"] = wrapper_data.decode("utf-8")
        return files
