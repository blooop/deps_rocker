import os
import pwd
import logging
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class CWDHome(SimpleRockerExtension):
    """Mount the current working directory inside the container's home directory and set it as the working directory"""

    name = "cwdhome"
    depends_on_extension = ("user",)

    def get_docker_args(self, cliargs) -> str:
        """
        Mount the current working directory inside the container's home directory
        at ~/project_name and set the working directory to that location.
        """
        # Get the container home directory from user extension
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir
        if not container_home:
            logging.warning(
                "Could not determine container home directory. Cannot mount CWD to home."
            )
            return ""

        # Get the current working directory and its name
        host_cwd = Path.cwd()
        project_name = host_cwd.name

        # Mount CWD inside container home (e.g., ~/projectA) and set working directory to it
        container_project_path = f"{container_home}/{project_name}"
        return f' -v "{host_cwd}:{container_project_path}" -w "{container_project_path}"'
