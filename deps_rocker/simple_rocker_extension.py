import pkgutil
import logging
import em
from rocker.extensions import RockerExtension
from typing import Type
from argparse import ArgumentParser
from typing import Dict, Optional


class SimpleRockerExtensionMeta(type):
    """Use a metaclass to dynamically create the static register_argument() function based on the class name and docstring"""

    def __new__(cls, name, bases, class_dict):
        # Create the class as usual
        new_class = super().__new__(cls, name, bases, class_dict)

        # Skip the base class itself
        if name != "BaseExtension":
            # Dynamically add the register_arguments method
            @staticmethod
            def register_arguments(parser: ArgumentParser, defaults: Optional[Dict] = None) -> None:
                new_class.register_arguments_helper(new_class, parser, defaults)

            new_class.register_arguments = register_arguments

        return new_class


class SimpleRockerExtension(RockerExtension, metaclass=SimpleRockerExtensionMeta):
    """A class to take care of most of the boilerplace required for a rocker extension"""

    name = "simple_rocker_extension"
    empy_args = {}
    empy_user_args = {}
    depends_on_extension: tuple[str, ...] = ()  # Tuple of dependencies required by the extension
    apt_packages: list[str] = []  # List of apt packages required by the extension

    @classmethod
    def get_name(cls) -> str:
        return cls.name

    def _get_pkg(self):
        # Dynamically determine the package/module path for the extension
        # e.g. 'deps_rocker.extensions.curl' for Curl
        module = self.__class__.__module__
        # If running as __main__, fallback to base package
        if module == "__main__":
            return "deps_rocker"
        return module

    def get_snippet(self, cliargs) -> str:
        snippet = self.get_and_expand_empy_template(self.empy_args)

        # If apt_packages is defined, generate apt install command
        if self.apt_packages:
            apt_snippet = self.get_apt_command(self.apt_packages, use_cache_mount=None)
            # If there's an existing snippet, append the apt command
            snippet = f"{apt_snippet}\n\n{snippet}" if snippet else apt_snippet
        return snippet

    def get_user_snippet(self, cliargs) -> str:
        return self.get_and_expand_empy_template(self.empy_user_args, snippet_prefix="user_")

    def get_and_expand_empy_template(self, empy_args, snippet_prefix=""):
        """
        Loads and expands an empy template snippet for Dockerfile generation.
        Args:
            empy_args: Arguments to expand in the template
            snippet_prefix: Prefix for the snippet name (default: "")
        Returns:
            Expanded template string or empty string if not found/error
        """
        snippet_name = f"{self.name}_{snippet_prefix}snippet.Dockerfile"
        try:
            pkg = self._get_pkg()
            dat = pkgutil.get_data(pkg, snippet_name)
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
        except Exception as e:
            logging.error(f"Error reading empy template: {e}")
        return ""

    @staticmethod
    def register_arguments(parser: ArgumentParser, defaults: dict = None):
        """This gets dynamically defined by the metaclass"""

    def get_config_file(self, path: str) -> Optional[bytes]:
        pkg = self._get_pkg()
        return pkgutil.get_data(pkg, path)

    @staticmethod
    def register_arguments_helper(
        class_type: Type, parser: ArgumentParser, defaults: dict = None
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
        # Ensure defaults is initialized as an empty dictionary if not provided
        if defaults is None:
            defaults = {}

        # Check if __doc__ is not None and has content
        if not class_type.__doc__:
            raise ValueError(
                f"The class '{class_type.__name__}' must have a docstring to use as the argument help text."
            )
        # Replace underscores with dashes in the class name for argument naming
        arg_name = class_type.name.replace("_", "-")

        # Add the argument to the parser
        parser.add_argument(
            f"--{arg_name}",
            action="store_true",
            default=defaults.get("deps_rocker"),
            help=class_type.__doc__,  # Use the class docstring as the help text
        )

    def invoke_after(self, cliargs: dict) -> set[str]:
        """
        Returns a set of dependencies that should be invoked after this extension.
        If deps is defined, returns it as a set.
        """
        return set(self.depends_on_extension) if self.depends_on_extension else set()

    def required(self, cliargs: dict) -> set[str]:
        """
        Returns a set of dependencies required by this extension.
        If deps is defined, returns it as a set.
        """
        return set(self.depends_on_extension) if self.depends_on_extension else set()

    @staticmethod
    def get_apt_command(packages: list[str], use_cache_mount: bool = None) -> str:
        """
        Generate an apt install command with optional cache mount for BuildKit.

        Args:
            packages: List of apt packages to install
            use_cache_mount: Whether to use BuildKit cache mounts (None=auto-detect, True=force, False=disable)

        Returns:
            Complete RUN command string for Dockerfile
        """
        if not packages:
            return ""

        packages_str = " \\\n    ".join(packages)

        # Auto-detect if we should use cache mounts based on environment
        if use_cache_mount is None:
            # Default to False for tests to maintain compatibility
            use_cache_mount = False

        if use_cache_mount:
            return f"""RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \\
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \\
    apt-get update && apt-get install -y --no-install-recommends \\
    {packages_str}"""
        return f"""RUN apt-get update && apt-get install -y --no-install-recommends \\
    {packages_str} \\
    && apt-get clean && rm -rf /var/lib/apt/lists/*"""
