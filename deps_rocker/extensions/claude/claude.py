import os
import pwd
import logging
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Claude(SimpleRockerExtension):
    """Install Claude Code via install script and mount host ~/.claude into the container"""

    name = "claude"
    # Ensure curl is available for the install script, and user exists for mounting into home
    depends_on_extension: tuple[str, ...] = ("curl", "user")

    def get_docker_args(self, cliargs) -> str:
        """
        Mount the host ~/.claude directory into the container user's home (~/.claude).
        If the host directory doesn't exist, skip mounting with a warning.
        """
        host_dir = os.path.expanduser("~/.claude")
        if not os.path.exists(host_dir):
            logging.warning(
                "Host ~/.claude directory does not exist. Claude config will not be mounted."
            )
            return ""

        # Attempt to get the container user's home dir from cliargs, fallback to host's home dir
        container_home = cliargs.get("user_home_dir") or pwd.getpwuid(os.getuid()).pw_dir
        if not container_home:
            logging.warning(
                "Could not determine container home directory. Skipping ~/.claude mount."
            )
            return ""

        target_path = f"{container_home}/.claude"
        return f' -v "{host_dir}:{target_path}"'
