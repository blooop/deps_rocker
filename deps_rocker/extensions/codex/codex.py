import logging
import os
import pwd
from pathlib import Path

from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Codex(SimpleRockerExtension):
    """Install OpenAI Codex CLI for AI-assisted development"""

    name = "codex"
    depends_on_extension = ("npm", "user")

    def _resolve_container_home(self, cliargs) -> str | None:
        """Resolve container home directory from cliargs and environment."""
        user_home = cliargs.get("user_home_dir")
        if user_home:
            return user_home
        return pwd.getpwuid(os.getuid()).pw_dir

    def get_docker_args(self, cliargs) -> str:
        """Mount host Codex config to reuse authentication inside the container."""
        container_home = self._resolve_container_home(cliargs)
        if not container_home:
            logging.warning(
                "Codex extension: unable to determine container home directory; skipping config mount."
            )
            return ""

        host_codex = (Path.home() / ".codex").expanduser().resolve()
        if not host_codex.is_dir():
            logging.warning(
                "Codex extension: no ~/.codex directory found on host; the CLI may prompt for login inside the container."
            )
            return ""

        container_codex = Path(container_home) / ".codex"
        args = [
            f'-v "{host_codex}:{container_codex}"',
        ]
        return " ".join(args)
