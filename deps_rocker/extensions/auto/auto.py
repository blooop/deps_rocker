"""Automatically detect and enable extensions based on workspace files"""

from rocker.extensions import RockerExtension
from pathlib import Path


class Auto(RockerExtension):
    """
    Detect project files and enable relevant extensions based on workspace contents.
    Use --auto=~/renv to specify the root directory for recursive search.
    """

    @classmethod
    def get_name(cls):
        return cls.name

    @staticmethod
    def register_arguments(parser, defaults=None):
        """
        Register command-line arguments for the auto extension.
        """
        parser.add_argument(
            "--auto",
            type=str,
            nargs="?",
            const=str(Path.cwd()),
            help="Enable auto extension and optionally specify a search root directory. Defaults to current working directory.",
        )

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
        print(f"[AUTO] Starting exact file match checks in: {workspace}")
        extensions |= self._detect_exact(workspace, file_patterns)

        # Data-driven exact directory checks
        dir_patterns = {
            ".gemini": "gemini",
            ".codex": "codex",
            ".claude": "claude",
        }
        print(f"[AUTO] Starting exact directory match checks in: {workspace}")
        extensions |= self._detect_exact_dir(workspace, dir_patterns)

        # requirements*.txt recursive
        print(f"[AUTO] Searching recursively for requirements*.txt in: {workspace}")
        extensions |= self._detect_glob(workspace, "requirements*.txt", "uv")

        # conda env files recursive
        print(f"[AUTO] Searching recursively for conda environment files in: {workspace}")
        extensions |= self._detect_conda(workspace)

        # C/C++ files
        print(f"[AUTO] Searching recursively for C/C++ files in: {workspace}")
        extensions |= self._detect_cpp(workspace)

        print(f"[AUTO] Final detected extensions: {extensions}")
        return extensions

    def _resolve_workspace(self, cliargs):
        root = cliargs.get("auto")
        # If root is True (from --auto with no value), treat as None
        if root is True or root is None:
            path = Path.cwd().expanduser().resolve()
        elif isinstance(root, (str, Path)):
            path = Path(root).expanduser().resolve()
        else:
            path = Path.cwd().expanduser().resolve()
        print(f"[AUTO] Scanning workspace: {path}")
        print(f"[AUTO] Workspace exists: {path.exists()}")
        print(f"[AUTO] Workspace is_dir: {path.is_dir()}")
        return path

    def _detect_exact(self, workspace, patterns):
        found = set()
        for fname, ext in patterns.items():
            file_path = workspace / fname
            if file_path.exists():
                print(f"[AUTO] ✓ Detected {fname} -> enabling {ext}")
                found.add(ext)
        return found

    def _detect_exact_dir(self, workspace, patterns):
        found = set()
        for dname, ext in patterns.items():
            dir_path = workspace / dname
            if dir_path.is_dir():
                print(f"[AUTO] ✓ Detected {dname} directory -> enabling {ext}")
                found.add(ext)
        return found

    def _detect_glob(self, workspace, pattern, ext):
        files = list(workspace.rglob(pattern))
        print(f"[AUTO] Recursively checking {pattern}: found {len(files)} files")
        if files:
            print(f"[AUTO] ✓ Detected {pattern} files -> enabling {ext}")
            return {ext}
        return set()

    def _detect_conda(self, workspace):
        env_files = list(workspace.rglob("environment.yml")) + list(
            workspace.rglob("environment.yaml")
        )
        print(
            f"[AUTO] Checking conda: found {len(env_files)} environment.yml/environment.yaml files"
        )
        if env_files:
            print("[AUTO] ✓ Detected conda environment file(s) -> enabling conda")
            return {"conda"}
        return set()

    def _detect_cpp(self, workspace):
        cpp_patterns = ["*.cpp", "*.hpp", "*.cc", "*.cxx", "*.h", "*.c", "*.hxx"]
        for pattern in cpp_patterns:
            if cpp_files := list(workspace.rglob(pattern)):
                print(f"[AUTO] ✓ Detected {len(cpp_files)} {pattern} files -> enabling ccache")
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
