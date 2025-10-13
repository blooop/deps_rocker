import os
import hashlib
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class RosJazzy(SimpleRockerExtension):
    """Adds ros-jazzy to your docker container"""

    name = "ros_jazzy"

    depends_on_extension = ("vcstool", "curl")
    # Use apt_packages feature for ROS dependencies
    apt_packages = [
        "locales",
        "tzdata",
        "curl",
        "gnupg2",
        "lsb-release",
        "sudo",
        "software-properties-common",
        "wget",
        "python3-pip",
        "cmake",
        "build-essential",
        "python3-argcomplete",
    ]

    def invoke_after(self, cliargs):
        return super().invoke_after(set(["gemini", "claude", "codex"]))

    def get_files(self, cliargs) -> dict[str, str]:
        dat = self.get_config_file("configs/colcon-defaults.yaml")

        # Get underlay build scripts
        script_dir = Path(__file__).parent
        underlay_deps = (script_dir / "underlay_deps.sh").read_text()
        underlay_build = (script_dir / "underlay_build.sh").read_text()

        return {
            "colcon-defaults.yaml": dat,
            "underlay_deps.sh": underlay_deps,
            "underlay_build.sh": underlay_build,
        }

    def get_docker_args(self, cliargs) -> str:
        """Set the ROS_DOMAIN_ID env var from the host machine if it exists, otherwise generate one from a hash of the username"""
        ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID")
        if ROS_DOMAIN_ID is None:
            username = os.environ.get("USER")
            if username:
                # Hash the username
                hashed_value = int(hashlib.sha256(username.encode()).hexdigest(), 16)
                # Scale the hash to a value between 2 and 99
                ROS_DOMAIN_ID = str((hashed_value % 98) + 2)
            else:
                raise ValueError("Unable to determine username and no ROS_DOMAIN_ID provided.")

        return f" --env ROS_DOMAIN_ID={ROS_DOMAIN_ID}"
