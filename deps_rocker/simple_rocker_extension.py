import pkgutil
import logging
import em
from pathlib import Path
from rocker.extensions import RockerExtension
from typing import Type
from argparse import ArgumentParser
from typing import Dict, Optional
from deps_rocker.buildkit import is_buildkit_enabled


def get_workspace_path() -> Path:
    """
    Get the workspace directory path.

    Returns the current working directory where rocker is invoked.
    This is typically where the user's project files are located.

    Returns:
        Path: The workspace directory path
    """
    return Path.cwd()


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

    @property
    def builder_output_dir(self):
        return f"/opt/deps_rocker/{self.name}"

    @property
    def builder_stage(self):
        return f"{self.name}_builder"

    def _with_builder_defaults(self, raw: dict) -> dict:
        out = dict(raw)
        out.setdefault("builder_output_dir", self.builder_output_dir)
        out.setdefault("builder_stage", self.builder_stage)
        return out

    @property
    def empy_args_with_builder(self):
        return self._with_builder_defaults(self.empy_args)

    @property
    def empy_builder_args_with_builder(self):
        return self._with_builder_defaults(self.empy_builder_args)

    name = "simple_rocker_extension"
    empy_args = {}
    empy_user_args = {}

    @property
    def empy_builder_args(self):
        # If someone overwrote empy_builder_args on the instance, use it; otherwise fall back to empy_args
        return getattr(self, "__empy_builder_args", self.empy_args)

    @empy_builder_args.setter
    def empy_builder_args(self, value: dict):
        # store directly on instance, avoids separate _empy_builder_args attr
        object.__setattr__(self, "__empy_builder_args", value)

    depends_on_extension: tuple[str, ...] = ()  # Tuple of dependencies required by the extension
    apt_packages: list[str] = []  # List of apt packages required by the extension
    builder_apt_packages: list[str] = []  # List of apt packages required in the builder stage
    builder_output_root = "/opt/deps_rocker"

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
        snippet = self.get_and_expand_empy_template(cliargs, self.empy_args)

        # If apt_packages is defined, generate apt install command
        if self.apt_packages:
            # Enable cache mounts when BuildKit is active
            apt_snippet = self.get_apt_command(
                self.apt_packages, use_cache_mount=is_buildkit_enabled()
            )
            # If there's an existing snippet, append the apt command
            snippet = f"{apt_snippet}\n\n{snippet}" if snippet else apt_snippet
        return snippet

    def get_user_snippet(self, cliargs) -> str:
        return self.get_and_expand_empy_template(
            cliargs, self.empy_user_args, snippet_prefix="user_"
        )

    def get_preamble(self, cliargs):
        fragments: list[str] = []

        if builder_snippet := self.get_builder_snippet(cliargs):
            fragments.append(builder_snippet)

        return "\n\n".join(fragments)

    def get_builder_snippet(self, cliargs) -> str:
        snippet = self.get_and_expand_empy_template(
            cliargs, getattr(self, "empy_builder_args", None), snippet_prefix="builder_"
        )

        # If builder_apt_packages is defined, generate apt install command and insert after FROM
        if self.builder_apt_packages and snippet:
            # Enable cache mounts when BuildKit is active
            apt_snippet = self.get_apt_command(
                self.builder_apt_packages, use_cache_mount=is_buildkit_enabled()
            )

            # Insert apt command after the FROM line
            lines = snippet.split("\n")
            insert_index = 0

            # Find the last line that starts with FROM, ARG, or is empty/comment after FROM
            for i, line in enumerate(lines):
                stripped = line.strip()
                if stripped.startswith("FROM ") or stripped.startswith('@(f"FROM'):
                    insert_index = i + 1
                elif insert_index > 0 and (
                    stripped.startswith("ARG ")
                    or stripped.startswith("ENV ")
                    or stripped.startswith("#")
                    or not stripped
                ):
                    insert_index = i + 1
                elif insert_index > 0:
                    # Found first non-FROM/ARG/ENV/comment/empty line, stop here
                    break

            # Insert the apt snippet at the determined position
            lines.insert(insert_index, "\n" + apt_snippet)
            snippet = "\n".join(lines)

        return snippet

    def get_and_expand_empy_template(self, cliargs, empy_args=None, snippet_prefix=""):
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
                template_args = self._build_template_args(cliargs, empy_args)
                logging.info(f"empy_{snippet_prefix}args: {template_args}")
                expanded = em.expand(snippet, template_args)
                logging.info(f"expanded\n{expanded}")
                return expanded
        except FileNotFoundError as _:
            logging.info(f"no user snippet found {snippet_name}")
        except Exception as e:
            self._log_empy_template_error(snippet_name, e)
        return ""

    def _log_empy_template_error(self, snippet_name, e):
        error_msg = (
            f"Error processing empy template '{snippet_name}' in extension '{self.name}': {e}"
        )

        # Provide specific guidance for common empy template errors
        if "unterminated string literal" in str(e).lower():
            error_msg += (
                "\n"
                + " " * 4
                + "HINT: This often occurs when using '@' or '$' characters in Dockerfile commands."
            )
            error_msg += (
                "\n"
                + " " * 4
                + "      In empy templates, escape '@' as '@@' and '$' as '$$' when they should be literal characters."
            )
            error_msg += (
                "\n"
                + " " * 4
                + "      Example: 'npm install -g package@version' should be 'npm install -g package@@version'"
            )
        elif "syntax error" in str(e).lower():
            error_msg += (
                "\n"
                + " " * 4
                + "HINT: Check for unescaped special characters in your Dockerfile snippet."
            )
            error_msg += (
                "\n"
                + " " * 4
                + "      Common issues: unescaped '@' or '$' characters, missing quotes, or malformed template syntax."
            )

        logging.error(error_msg)

    def _build_template_args(self, cliargs, empy_args=None) -> dict:
        args = empy_args.copy() if empy_args else {}
        base_image = cliargs.get("base_image", "") if cliargs else ""
        args |= {
            "base_image": base_image,
            "builder_stage": self.get_builder_stage_name(),
            "builder_output_dir": self.get_builder_output_dir(),
            "builder_output_path": f"{self.get_builder_output_dir()}/",
            "extension_name": self.name,
        }
        return args

    def get_builder_stage_name(self) -> str:
        return f"{self.name}_builder"

    def get_builder_output_dir(self) -> str:
        return f"{self.builder_output_root}/{self.name}"

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

    # alias the module-level function to avoid duplication
    get_workspace_path = staticmethod(get_workspace_path)

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

        # Default to automatic detection when not explicitly provided
        if use_cache_mount is None:
            use_cache_mount = is_buildkit_enabled()

        if use_cache_mount:
            return f"""RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \\
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \\
    apt-get update && apt-get install -y --no-install-recommends \\
    {packages_str}"""
        return f"""RUN apt-get update && apt-get install -y --no-install-recommends \\
    {packages_str} \\
    && apt-get clean && rm -rf /var/lib/apt/lists/*"""
