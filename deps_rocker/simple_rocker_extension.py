import pkgutil
import logging
from rocker.extensions import RockerExtension


class SimpleRockerExtension(RockerExtension):
    name = "simple_rocker_extension"
    pkg = "deps_rocker"

    @classmethod
    def get_name(cls):
        return cls.name

    def get_snippet(self, cliargs):
        try:
            dat = pkgutil.get_data(self.pkg, f"templates/{self.name}_snippet.Dockerfile")
            if dat is not None:
                return dat.decode("utf-8")
        except FileNotFoundError as e:
            logging.info(f"no snippet found templates/{self.name}_snippet.Dockerfile")
        return ""

    def get_user_snippet(self, cliargs):
        try:
            dat = pkgutil.get_data(self.pkg, f"templates/{self.name}_snippet_user.Dockerfile")
            if dat is not None:
                return dat.decode("utf-8")
        except FileNotFoundError as e:
            logging.info(f"no snippet found templates/{self.name}_user_snippet.Dockerfile")
        return ""

    @staticmethod
    def register_arguments_helper(name: str, parser, defaults=None):
        arg_name = name.replace("_", "-")
        docs_name = name.replace("_", " ")
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{arg_name}",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help=f"add {docs_name} to your docker image",
        )
