import pkgutil
import logging
import em
from rocker.extensions import RockerExtension
from typing import Type
from argparse import ArgumentParser

class SimpleRockerExtension(RockerExtension):
    name = "simple_rocker_extension"
    pkg = "deps_rocker"
    empy_args = {}
    empy_user_args = {}

    @classmethod
    def get_name(cls) -> str:
        return cls.name

    def get_snippet(self, cliargs) -> str:
        return self.get_and_expand_empy_template(self.empy_args)

    def get_user_snippet(self, cliargs) -> str:
        return self.get_and_expand_empy_template(
            self.empy_user_args,
            "user_",
        )

    def get_and_expand_empy_template(
        self,
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
    def register_arguments_helper(
        class_type: Type, 
        parser: ArgumentParser, 
        defaults: dict = None
    ) -> None:
        """
        Registers arguments for a given class type to an `ArgumentParser` instance.

        Args:
            class_type (Type): The class whose name and docstring are used to define the argument.
                               The class must have a `name` attribute (str) and a docstring.
            parser (ArgumentParser): The `argparse.ArgumentParser` object to which the argument is added.
            defaults (dict): A dictionary of default values for the arguments.
                                                            If `None`, defaults to an empty dictionary.

        Returns:
            None: This method does not return any value. It modifies the `parser` in place.

        Raises:
            AttributeError: If the `class_type` does not have a `name` attribute.
        """
        # Replace underscores with dashes in the class name for argument naming
        arg_name = class_type.name.replace("_", "-")
        
        # Ensure defaults is initialized as an empty dictionary if not provided
        if defaults is None:
            defaults = {}

        assert(len(class_type.__doc__)>0)

        # Add the argument to the parser
        parser.add_argument(
            f"--{arg_name}",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help=class_type.__doc__,  # Use the class docstring as the help text
        )