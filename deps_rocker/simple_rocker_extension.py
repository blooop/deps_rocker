import pkgutil
import logging
from rocker.extensions import RockerExtension
import em


class SimpleRockerExtension(RockerExtension):
    name = "simple_rocker_extension"
    pkg = "deps_rocker"
    empy_args = {}
    empy_user_args = {}

    @classmethod
    def get_name(cls):
        return cls.name

    def get_snippet(self, cliargs):
        try:
            dat = pkgutil.get_data(self.pkg, f"templates/{self.name}_snippet.Dockerfile")
            if dat is not None:
                snippet = dat.decode("utf-8")
                logging.info(f"empy_snippet: {snippet}")
                logging.info(f"empy_args: {self.empy_args}")
                expanded = em.expand(snippet, self.empy_args)
                logging.info(f"expanded\n{expanded}")
                return expanded
        except FileNotFoundError as _:
            logging.info(f"no snippet found templates/{self.name}_snippet.Dockerfile")
        return ""

    def get_user_snippet(self, cliargs):
        try:
            dat = pkgutil.get_data(self.pkg, f"templates/{self.name}_suer_snippet.Dockerfile")
            if dat is not None:
                snippet = dat.decode("utf-8")
                logging.info(f"empy_user_snippet: {snippet}")
                logging.info(f"empy_user_args: {self.empy_user_args}")
                expanded = em.expand(snippet, self.empy_user_args)
                logging.info(f"expanded\n{expanded}")
                return expanded
        except FileNotFoundError as _:
            logging.info(f"no user snippet found templates/{self.name}_user_snippet.Dockerfile")
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
