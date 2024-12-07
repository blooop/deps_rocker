import pkgutil
from rocker.extensions import RockerExtension


class SimpleRockerExtension(RockerExtension):
    name = "simple_rocker_extension"
    pkg = "deps_rocker"

    @classmethod
    def get_name(cls):
        return cls.name

    def get_snippet(self, cliargs):
        return pkgutil.get_data(self.pkg, f"templates/{self.name}_snippet.Dockerfile").decode(
            "utf-8"
        )

    def get_user_snippet(self, cliargs):
        return pkgutil.get_data(self.pkg, f"templates/{self.name}_snippet_user.Dockerfile").decode(
            "utf-8"
        )

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            "--ros-humble",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help="add ros humble to your docker image",
        )
