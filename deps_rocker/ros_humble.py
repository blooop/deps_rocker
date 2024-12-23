import os
import hashlib
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class RosHumble(SimpleRockerExtension):
    """Adds ros-humble to your docker container"""

    name = "ros_humble"

    def invoke_after(self, cliargs):
        return {"vcstool"}

    def required(self, cliargs):
        return {"vcstool"}

    def get_files(self, cliargs) -> dict[str, str]:
        dat = self.get_config_file("configs/ros_humble/defaults.yaml")
        return {"defaults.yaml": dat}

    def get_docker_args(self, cliargs) -> str:
        """Set the ROS_DOMAIN_ID env var from the host machine if it exists, otherwise generate one from a hash of the username"""
        ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID")
        if ROS_DOMAIN_ID is None:
            username = os.environ.get("USER")
            if username:
                # Hash the username
                hashed_value = int(hashlib.sha256(username.encode()).hexdigest(), 16)
                # Scale the hash to a value between 1 and 99
                ROS_DOMAIN_ID = str((hashed_value % 99) + 1)
            else:
                raise ValueError("Unable to determine username and no ROS_DOMAIN_ID provided.")

        return f"--env ROS_DOMAIN_ID={ROS_DOMAIN_ID}"
