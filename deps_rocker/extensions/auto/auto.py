import logging
from deps_rocker.simple_rocker_extension import SimpleRockerExtension
from pathlib import Path


class Auto(SimpleRockerExtension):
    """
    Detect project files and enable relevant extensions based on workspace contents.
    Use --auto-search-root to specify the root directory for recursive search.
    """

    @staticmethod
    def register_arguments(parser, defaults=None):
        """
        Register command-line arguments for the auto extension.
        """
        parser.add_argument(
            "--auto-search-root",
            type=str,
            default=None,
            help="Directory to start recursive search for project files (overrides default root)",
        )

    """Automatically detect and enable extensions based on workspace files"""

    name = "auto"

    def _detect_files_in_workspace(self, _cliargs: dict) -> set[str]:
        """
        Detect files in the workspace and return a set of extension names to enable.

        Args:
            _cliargs: CLI arguments dict (not used, kept for compatibility)

        Returns:
            Set of extension names to enable
        """
        extensions = set()

        # Use --auto-search-root argument if provided, else cwd
        cli_search_root = _cliargs.get("auto_search_root")
        if cli_search_root:
            workspace = Path(cli_search_root).expanduser().resolve()
        else:
            workspace = Path.cwd().expanduser().resolve()

        logging.warning(f"[AUTO] Scanning workspace: {workspace}")
        logging.warning(f"[AUTO] Workspace exists: {workspace.exists()}")
        logging.warning(f"[AUTO] Workspace is_dir: {workspace.is_dir()}")

        # Exact file matches
        file_patterns = {
            "pixi.toml": "pixi",
            "pyproject.toml": "uv",
            ".python-version": "uv",
            "poetry.lock": "uv",
            "package.json": "npm",
            "Cargo.toml": "cargo",
            "package.xml": "ros_jazzy",
        }

        for filename, extension in file_patterns.items():
            file_path = workspace / filename
            exists = file_path.exists()
            logging.warning(f"[AUTO] Checking {filename}: {file_path} -> exists={exists}")
            if exists:
                logging.warning(f"[AUTO] ✓ Detected {filename} -> enabling {extension}")
                extensions.add(extension)

        # Pattern-based matches (requirements*.txt)
        req_files = list(workspace.glob("requirements*.txt"))
        logging.warning(f"[AUTO] Checking requirements*.txt: found {len(req_files)} files")
        if req_files:
            logging.warning("[AUTO] ✓ Detected requirements files -> enabling uv")
            extensions.add("uv")

        # Check for conda environment files
        env_yml = (workspace / "environment.yml").exists()
        env_yaml = (workspace / "environment.yaml").exists()
        logging.warning(
            f"[AUTO] Checking conda: environment.yml={env_yml}, environment.yaml={env_yaml}"
        )
        if env_yml or env_yaml:
            logging.warning("[AUTO] ✓ Detected conda environment file -> enabling conda")
            extensions.add("conda")

        # Check for C/C++ files (for ccache)
        cpp_patterns = ["*.cpp", "*.hpp", "*.cc", "*.cxx", "*.h", "*.c", "*.hxx"]
        for pattern in cpp_patterns:
            cpp_files = list(workspace.rglob(pattern))
            if cpp_files:
                logging.warning(
                    f"[AUTO] ✓ Detected {len(cpp_files)} {pattern} files -> enabling ccache"
                )
                extensions.add("ccache")
                break  # Found at least one C++ file, no need to continue

        logging.warning(f"[AUTO] Final detected extensions: {extensions}")
        return extensions

    def required(self, cliargs: dict) -> set[str]:
        """
        Returns a set of dependencies required by this extension based on detected files.

        Args:
            cliargs: CLI arguments dict

        Returns:
            Set of extension names to enable
        """
        return self._detect_files_in_workspace(cliargs)
