import pkgutil
from rocker.extensions import RockerExtension


class RosHumble(RockerExtension):
    @staticmethod
    def get_name():
        return "ros-humble"

    def __init__(self):
        self.name = RosHumble.get_name()

    def get_snippet(self, cliargs):
        return pkgutil.get_data("ros_humble", "templates/ros-humble_snippet.Dockerfile").decode(
            "utf-8"
        )

    def get_user_snippet(self, cliargs):
        return pkgutil.get_data(
            "ros_humble", "templates/{}_snippet.Dockerfile".format(self.name)
        ).decode("utf-8")

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{RosHumble.get_name()}",
            action="store_true",
            default=defaults.get("ros_humble"),
            help="add ros-humble to your docker image",
        )
