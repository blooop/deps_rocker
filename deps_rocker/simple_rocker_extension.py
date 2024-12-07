import pkgutil
import logging
import em
from rocker.extensions import RockerExtension


class SimpleRockerExtension(RockerExtension):
    name = "simple_rocker_extension"
    pkg = "deps_rocker"
    empy_args = {}
    empy_user_args = {}

    @classmethod
    def get_name(cls) -> str:
        return cls.name

    def get_snippet(self, cliargs) -> str:
        return self.get_and_expand_empy_template(cliargs, self.empy_args)

    def get_user_snippet(self, cliargs) -> str:
        return self.get_and_expand_empy_template(
            cliargs,
            self.empy_user_args,
            "user_",
        )

    def get_and_expand_empy_template(
        self,
        cliargs,
        empy_args,
        snippet_prefix: str = "",
    ) -> str:
        try:
            snippet_name = f"templates/{self.name}_{snippet_prefix}snippet.Dockerfile"
            dat = pkgutil.get_data(self.pkg, snippet_name)
            if dat is not None:
                snippet = dat.decode("utf-8")
                logging.warning(self.name)
                logging.info(f"empy_{snippet_prefix}snippet: {snippet}")
                logging.info(f"empy_{snippet_prefix}args: {empy_args}")
                expanded = em.expand(snippet, empy_args)
                logging.info(f"expanded\n{expanded}")
                return expanded
        except FileNotFoundError as _:
            logging.info(f"no user snippet found {snippet_name}")
        return ""

    @staticmethod
    def register_arguments(parser, defaults=None):
        raise NotImplementedError

    @staticmethod
    def register_arguments_helper(name: str, parser, defaults=None) -> None:
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
