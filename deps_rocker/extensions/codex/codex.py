import logging
import os
import pwd

from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Codex(SimpleRockerExtension):
    """Install OpenAI Codex CLI for AI-assisted development"""

    name = "codex"
    depends_on_extension = ("npm", "user")

    def get_docker_args(self, cliargs) -> str:
        """Mount host Codex config to reuse authentication inside the container."""

        container_home = (
            cliargs.get("user_home_dir")
            or os.environ.get("DEPS_ROCKER_CONTAINER_HOME")
            or pwd.getpwuid(os.getuid()).pw_dir
        )
        if not container_home:
            logging.warning(
                "Codex extension: unable to determine container home directory; skipping config mount."
            )
            return ""

        host_codex_home = os.path.expanduser("~/.codex")
        if not os.path.exists(host_codex_home):
            logging.warning(
                "Codex extension: no ~/.codex directory found on host; the CLI may prompt for login inside the container."
            )
            return ""

        host_codex_home = os.path.realpath(host_codex_home)
        container_codex_home = f"{container_home}/.codex"

        docker_args = [
            f' -v "{host_codex_home}:{container_codex_home}"',
            f' -e "CODEX_HOME={container_codex_home}"',
        ]

        return "".join(docker_args)
