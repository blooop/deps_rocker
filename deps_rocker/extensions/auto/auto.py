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
        """
        workspace = self._resolve_workspace(_cliargs)
        extensions = set()

        # Data-driven exact file checks
        file_patterns = {
            "pixi.toml": "pixi",
            "pyproject.toml": "uv",
            ".python-version": "uv",
            "poetry.lock": "uv",
            "package.json": "npm",
            "Cargo.toml": "cargo",
            "package.xml": "ros_jazzy",
        }
        extensions |= self._detect_exact(workspace, file_patterns)

        # requirements*.txt recursive
        extensions |= self._detect_glob(workspace, "requirements*.txt", "uv")

        # conda env files recursive
        extensions |= self._detect_conda(workspace)

        # C/C++ files
        extensions |= self._detect_cpp(workspace)

        logging.info(f"[AUTO] Final detected extensions: {extensions}")
        return extensions

    def _resolve_workspace(self, cliargs):
        root = cliargs.get("auto_search_root")
        path = Path(root).expanduser().resolve() if root else Path.cwd().expanduser().resolve()
        logging.info(f"[AUTO] Scanning workspace: {path}")
        logging.info(f"[AUTO] Workspace exists: {path.exists()}")
        logging.info(f"[AUTO] Workspace is_dir: {path.is_dir()}")
        return path

    def _detect_exact(self, workspace, patterns):
        found = set()
        for fname, ext in patterns.items():
            file_path = workspace / fname
            if file_path.exists():
                logging.info(f"[AUTO] ✓ Detected {fname} -> enabling {ext}")
                found.add(ext)
        return found

    def _detect_glob(self, workspace, pattern, ext):
        files = list(workspace.rglob(pattern))
        logging.info(f"[AUTO] Recursively checking {pattern}: found {len(files)} files")
        if files:
            logging.info(f"[AUTO] ✓ Detected {pattern} files -> enabling {ext}")
            return {ext}
        return set()

    def _detect_conda(self, workspace):
        env_files = list(workspace.rglob("environment.yml")) + list(
            workspace.rglob("environment.yaml")
        )
        logging.info(
            f"[AUTO] Checking conda: found {len(env_files)} environment.yml/environment.yaml files"
        )
        if env_files:
            logging.info("[AUTO] ✓ Detected conda environment file(s) -> enabling conda")
            return {"conda"}
        return set()

    def _detect_cpp(self, workspace):
        cpp_patterns = ["*.cpp", "*.hpp", "*.cc", "*.cxx", "*.h", "*.c", "*.hxx"]
        for pattern in cpp_patterns:
            if cpp_files := list(workspace.rglob(pattern)):
                logging.info(
                    f"[AUTO] ✓ Detected {len(cpp_files)} {pattern} files -> enabling ccache"
                )
                return {"ccache"}
        return set()

    def required(self, cliargs: dict) -> set[str]:
        """
        Returns a set of dependencies required by this extension based on detected files.

        Args:
            cliargs: CLI arguments dict

        Returns:
            Set of extension names to enable
        """
        return self._detect_files_in_workspace(cliargs)
