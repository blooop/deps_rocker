from deps_rocker.simple_rocker_extension import SimpleRockerExtension
from argparse import ArgumentParser
from typing import Dict, Optional


class Workdir(SimpleRockerExtension):
    """Set the working directory in the Docker container"""

    name = "workdir"

    @staticmethod
    def register_arguments(parser: ArgumentParser, defaults: Optional[Dict] = None) -> None:
        if defaults is None:
            defaults = {}

        parser.add_argument(
            "--workdir",
            type=str,
            default=defaults.get("workdir"),
            metavar="PATH",
            help="Set the working directory in the Docker container",
        )

    def get_snippet(self, cliargs) -> str:
        workdir = getattr(cliargs, "workdir", None)
        if workdir:
            return f"WORKDIR {workdir}"
        return ""
