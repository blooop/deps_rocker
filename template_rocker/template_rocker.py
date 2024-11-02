import pkgutil
from rocker.extensions import RockerExtension


class TemplateRocker(RockerExtension):
    @staticmethod
    def get_name():
        return "template_rocker"

    def __init__(self):
        self.name = TemplateRocker.get_name()

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
            f"--{TemplateRocker.get_name()}",
            action="store_true",
            default=defaults.get("template_rocker"),
            help="add template_rocker to your docker image",
        )
