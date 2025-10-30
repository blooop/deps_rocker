import os
import pwd
import logging
from pathlib import Path
import re
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class CWD(SimpleRockerExtension):
    """Mount the current working directory inside the container's home directory at ~/project_name"""

    name = "cwd"
    depends_on_extension = ("user",)

    def get_docker_args(self, cliargs) -> str:
        """
        Mount the current working directory inside the container's home directory
        at ~/project_name and set the working directory to that location.
        Also runs the container as the user (not root).
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

        # Get user ID and group ID to run container as the user
        uid = os.getuid()
        gid = os.getgid()

        # Mount CWD inside container home (e.g., ~/projectA), set working directory, and run as user
        container_project_path = f"{container_home}/{project_name}"
        return f' -u {uid}:{gid} -v "{host_cwd}:{container_project_path}" -w "{container_project_path}"'

    def invoke_after(self, cliargs) -> set:
        return {"user"}


class CWDName(SimpleRockerExtension):
    """Set the name of the container to the name of the folder of the current working directory"""

    name = "cwd_name"

    def get_docker_args(self, cliargs) -> str:
        return f" --name {Path.cwd().stem}"

    @staticmethod
    def sanitize_container_name(name: str) -> str:
        """
        Sanitizes the container name to conform to Docker's requirements.

        Args:
            name (str): The original name, typically from the current working directory.

        Returns:
            str: A sanitized name that only includes alphanumeric characters, dots, and dashes.

        Raises:
            ValueError: If the sanitized name is empty after sanitization.
        """
        # Replace invalid characters with dashes
        sanitized = re.sub(r"[^a-zA-Z0-9.-]", "-", name)

        # Ensure the name is not empty after sanitization
        if not sanitized.strip("-"):
            raise ValueError(
                f"The sanitized container name '{name}' is invalid. Ensure the working directory name contains valid characters."
            )

        return sanitized
