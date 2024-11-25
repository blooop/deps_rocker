import pkgutil
from rocker.extensions import RockerExtension


class RosHumble(RockerExtension):
    @staticmethod
    def get_name():
        return "roshumble"

    def __init__(self):
        self.name = RosHumble.get_name()

    def get_snippet(self, cliargs):
        return pkgutil.get_data("deps_rocker", "templates/ros_humble_snippet.Dockerfile").decode(
            "utf-8"
        )

    def get_user_snippet(self, cliargs):
        return pkgutil.get_data(
            "deps_rocker", "templates/ros_humble_snippet_user.Dockerfile"
        ).decode("utf-8")

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{RosHumble.get_name()}",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help="add ros humble to your docker image",
        )
