import pkgutil
from rocker.extensions import RockerExtension


class IsaacSim(RockerExtension):
    @staticmethod
    def get_name():
        return "isaac-sim"

    def __init__(self):
        self.name = IsaacSim.get_name()

    def get_snippet(self, cliargs):
        return pkgutil.get_data("deps_rocker", f"templates/{self.name}_snippet.Dockerfile").decode(
            "utf-8"
        )

    # def get_user_snippet(self, cliargs):
    #     return pkgutil.get_data("deps_rocker", f"templates/{self.name}_user.Dockerfile").decode(
    #         "utf-8"
    #     )

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{IsaacSim.get_name()}",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help="add isaac_sim to your docker image",
        )
