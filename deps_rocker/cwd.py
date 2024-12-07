import pkgutil
from pathlib import Path
from rocker.extensions import RockerExtension


class CWD(RockerExtension):
    name = "cwd"

    @classmethod
    def get_name(cls):
        return cls.name

    def get_docker_args(self, cliargs):
        return " -v %s:%s " % (Path.home(), Path.cwd())

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{CWD.name}",
            action="store_true",
            help=f"add {CWD.name} to your docker image",
        )
