from pathlib import Path
import re
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class CWD(SimpleRockerExtension):
    """Add the current working directory as a volume in your docker container"""

    name = "cwd"

    def get_docker_args(self, cliargs) -> str:
        return " -v %s:%s " % (Path.cwd(), "/workspaces")

    def invoke_after(self, cliargs) -> set:
        # return set(["vcstool"])
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
