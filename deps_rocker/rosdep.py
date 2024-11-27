import pkgutil
from rocker.extensions import RockerExtension


class Rosdep(RockerExtension):
    @staticmethod
    def get_name():
        return "rosdep"

    def __init__(self):
        self.name = Rosdep.get_name()

    def get_snippet(self, cliargs):
        return pkgutil.get_data("deps_rocker", f"templates/{self.name}_snippet.Dockerfile").decode(
            "utf-8"
        )

    def get_user_snippet(self, cliargs):
        return pkgutil.get_data(
            "deps_rocker", f"templates/{self.name}_snippet_user.Dockerfile"
        ).decode("utf-8")

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{Rosdep.get_name()}",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help="add ros humble to your docker image",
        )
