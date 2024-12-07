from pathlib import Path
from rocker.extensions import RockerExtension


class CWD(RockerExtension):
    """Add the current working directory as a volume in your docker container"""

    name = "cwd"

    @classmethod
    def get_name(cls):
        return cls.name

    def get_docker_args(self, cliargs):
        return " -v %s:%s " % (Path.cwd(), Path.home())

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{CWD.name}",
            action="store_true",
            help=f"add {CWD.name} to your docker image",
        )
