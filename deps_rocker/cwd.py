from pathlib import Path
import pkgutil
from rocker.extensions import RockerExtension


class CWD(RockerExtension):
    """Add the current working directory as a volume in your docker container"""

    name = "cwd"

    @classmethod
    def get_name(cls):
        return cls.name

    def get_docker_args(self, cliargs):
        return " -v %s:%s " % (Path.cwd(), "/workspaces")

    def get_user_snippet(self, cliargs):
        snippet = pkgutil.get_data(
            "deps_rocker", f"templates/{self.name}_user_snippet.Dockerfile"
        ).decode("utf-8")

        return snippet

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{CWD.name}",
            action="store_true",
            help=f"add {CWD.name} to your docker image",
        )
