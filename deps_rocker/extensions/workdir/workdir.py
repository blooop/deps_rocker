import os
import pwd
import logging
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Workdir(SimpleRockerExtension):
    """Mount a host directory to a specified container location and set working directory"""

    name = "workdir"
    depends_on_extension = ("user",)

    @staticmethod
    def register_arguments(parser, defaults=None):
        """Register command-line arguments for the workdir extension."""
        parser.add_argument(
            "--workdir-host-path",
            type=str,
            default=None,
            help="Host directory to mount (defaults to current working directory)",
        )
        parser.add_argument(
            "--workdir-container-path",
            type=str,
            default=None,
            help="Container path to mount the host directory to (defaults to ~/project_name)",
        )
        parser.add_argument(
            "--workdir-working-dir",
            type=str,
            default=None,
            help="Working directory inside the container (defaults to mount path)",
        )

    def get_docker_args(self, cliargs) -> str:
        """
        Mount the specified host directory to the container and set the working directory.
        """
        # Get the container home directory from user extension
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir
        if not container_home:
            logging.warning("Could not determine container home directory. Cannot mount workdir.")
            return ""

        # Get host path (default to current working directory)
        host_path_str = cliargs.get("workdir_host_path")
        if host_path_str:
            host_path = Path(host_path_str).resolve()
        else:
            host_path = Path.cwd()

        if not host_path.exists():
            logging.warning(f"Host path {host_path} does not exist. Cannot mount workdir.")
            return ""

        # Get container mount path (default to ~/project_name)
        container_path = cliargs.get("workdir_container_path")
        if not container_path:
            project_name = host_path.name
            container_path = f"{container_home}/{project_name}"

        # Get working directory (default to mount path)
        working_dir = cliargs.get("workdir_working_dir")
        if not working_dir:
            working_dir = container_path

        # Build docker args
        args = f' -v "{host_path}:{container_path}" -w "{working_dir}"'

        return args

    def invoke_after(self, cliargs) -> set:
        return {"user"}

    def get_snippet(self, cliargs) -> str:
        """
        Add a Dockerfile snippet to create mount point with proper permissions.
        """
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir

        # Get container mount path (default to ~/project_name)
        container_path = cliargs.get("workdir_container_path")
        if not container_path:
            host_path_str = cliargs.get("workdir_host_path")
            if host_path_str:
                host_path = Path(host_path_str).resolve()
            else:
                host_path = Path.cwd()
            project_name = host_path.name
            container_path = f"{container_home}/{project_name}"

        return f"""
# Set up mount point for workdir with proper permissions
# Create the directory structure with world-writable permissions
RUN mkdir -p "{container_path}" && \\
    chmod 777 "{container_path}"
"""
