import os
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Foxglove(SimpleRockerExtension):
    """Install Foxglove Studio for robotics data visualization"""

    name = "foxglove"
    depends_on_extension = ("curl",)
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
    ]

    def get_docker_args(self, cliargs) -> str:
        """
        Mount Foxglove Agent persistent storage:
        - Named volume for agent index: foxglove-agent-index:/index
        - Host directory for recordings: ${HOME}/foxglove_recordings:/storage
        """
        home_dir = os.path.expanduser("~")
        recordings_dir = os.path.join(home_dir, "foxglove_recordings")

        # Create recordings directory if it doesn't exist
        os.makedirs(recordings_dir, exist_ok=True)

        return f' -v foxglove-agent-index:/index -v "{recordings_dir}:/storage"'
