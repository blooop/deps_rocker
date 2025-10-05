from deps_rocker.simple_rocker_extension import SimpleRockerExtension
import os
import hashlib


class RosGeneric(SimpleRockerExtension):
    """Adds a configurable ROS 2 distribution (default: humble) to your docker container"""

    name = "ros_generic"
    depends_on_extension = ("vcstool", "locales", "tzdata", "curl")

    def get_files(self, cliargs) -> dict[str, str]:
        dat = self.get_config_file("configs/defaults.yaml")
        return {"defaults.yaml": dat}

    def get_ros_distro(self, cliargs):
        # Allow override via cliargs, else default to humble
        return cliargs.get("ros_distro", "humble")

    def get_docker_args(self, cliargs) -> str:
        """Return a string of --env args for Docker run, space-separated."""
        ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID")
        if ROS_DOMAIN_ID is None:
            username = os.environ.get("USER")
            if username:
                hashed_value = int(hashlib.sha256(username.encode()).hexdigest(), 16)
                ROS_DOMAIN_ID = str((hashed_value % 99) + 1)
            else:
                raise ValueError("Unable to determine username and no ROS_DOMAIN_ID provided.")
        return (
            f" --env ROS_DOMAIN_ID={ROS_DOMAIN_ID} --env ROS_DISTRO={self.get_ros_distro(cliargs)}"
        )
