import os
import pkgutil
import logging
import shutil
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Foxglove(SimpleRockerExtension):
    """Install Foxglove Studio for robotics data visualization"""

    name = "foxglove"
    # Always require curl; attempt x11 only when the host can support it
    depends_on_extension = ("curl",)
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

        Note: Browser integration is provided via the x11 dependency for GUI forwarding when available.
        """
        home_dir = os.path.expanduser("~")
        recordings_dir = os.path.join(home_dir, "foxglove_recordings")

        # Create recordings directory if it doesn't exist
        os.makedirs(recordings_dir, exist_ok=True)

        return f' -v foxglove-agent-index:/index -v "{recordings_dir}:/storage"'

    def _supports_x11(self) -> bool:
        """
        Determine whether the host can satisfy the x11 precondition.
        Skip x11 when DISPLAY or xauth are unavailable (common in headless CI).
        """
        display = os.getenv("DISPLAY")
        if not display:
            logging.warning("foxglove: DISPLAY not set, skipping x11 dependency")
            return False
        if shutil.which("xauth") is None:
            logging.warning("foxglove: xauth not found on host, skipping x11 dependency")
            return False
        return True

    def _dependencies(self) -> set[str]:
        deps = set(self.depends_on_extension)
        if self._supports_x11():
            deps.add("x11")
        return deps

    def required(self, cliargs) -> set[str]:
        return self._dependencies()

    def invoke_after(self, cliargs) -> set[str]:
        return self._dependencies()

    def get_files(self, cliargs) -> dict[str, str]:
        files = super().get_files(cliargs) or {}
        wrapper_data = pkgutil.get_data(__name__, "foxglove_wrapper.sh")
        if wrapper_data is None:
            raise FileNotFoundError("foxglove_wrapper.sh not found in package data")
        files["foxglove_wrapper.sh"] = wrapper_data.decode("utf-8")
        return files
