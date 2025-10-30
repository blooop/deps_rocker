import os
import pwd
import logging
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class CWDHome(SimpleRockerExtension):
    """Mount the current working directory into the container's home directory and set it as the working directory"""

    name = "cwdhome"
    depends_on_extension = ("user",)

    def get_docker_args(self, cliargs) -> str:
        """
        Mount the current working directory into the container's home directory
        and set the working directory to the home directory.
        """
        # Get the container home directory from user extension
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir
        if not container_home:
            logging.warning(
                "Could not determine container home directory. Cannot mount CWD to home."
            )
            return ""

        # Get the current working directory
        host_cwd = Path.cwd()

        # Mount CWD to container home and set working directory
        return f' -v "{host_cwd}:{container_home}" -w "{container_home}"'
