import pkgutil
from rocker.extensions import RockerExtension


class VCStool(RockerExtension):
    @staticmethod
    def get_name():
        return "vcstool"

    def __init__(self):
        self.name = VCStool.get_name()

    # def get_snippet(self, cliargs):
    #     return "RUN pip install vcstool"

    def get_snippet(self, cliargs):
        return pkgutil.get_data(
            "deps_rocker", "templates/{}_snippet.Dockerfile".format(self.name)
        ).decode("utf-8")

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{VCStool.get_name()}",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help="import and install repos with vcstool in your docker image",
        )
