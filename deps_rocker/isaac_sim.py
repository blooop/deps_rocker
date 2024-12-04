import pkgutil
from typing import Set
from rocker.extensions import RockerExtension


class IsaacSim(RockerExtension):
    name = "isaac_sim"

    @classmethod
    def get_name(cls):
        return cls.name

    def get_snippet(self, cliargs):
        return pkgutil.get_data("deps_rocker", f"templates/{self.name}_snippet.Dockerfile").decode(
            "utf-8"
        )

    # def get_user_snippet(self, cliargs):
    #     return pkgutil.get_data("deps_rocker", f"templates/{self.name}_user.Dockerfile").decode(
    #         "utf-8"
    #     )

    def required(self, cliargs) -> Set[str]:
        return set(["nvidia", "privileged", "x11"])

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--isaac-sim",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help="add isaac_sim to your docker image",
        )
