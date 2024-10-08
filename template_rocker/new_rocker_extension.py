import pkgutil
from rocker.extensions import RockerExtension


class NewRockerExtension(RockerExtension):
    @staticmethod
    def get_name():
        return "new_rocker_extension"

    def __init__(self):
        self.name = NewRockerExtension.get_name()

    def get_snippet(self, cliargs):
        return pkgutil.get_data("template_rocker", "templates/curl_snippet.Dockerfile").decode(
            "utf-8"
        )

    def get_user_snippet(self, cliargs):
        return pkgutil.get_data(
            "template_rocker", "templates/{}_snippet.Dockerfile".format(self.name)
        ).decode("utf-8")

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{NewRockerExtension.get_name()}",
            action="store_true",
            default=defaults.get("new_rocker_extension"),
            help="add new_rocker_extension to your docker image",
        )
