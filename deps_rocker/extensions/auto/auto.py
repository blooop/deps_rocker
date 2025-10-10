from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Auto(SimpleRockerExtension):
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

        # Use centralized workspace path getter
        workspace = self.get_workspace_path()

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
            if (workspace / filename).exists():
                extensions.add(extension)

        # Pattern-based matches (requirements*.txt)
        if list(workspace.glob("requirements*.txt")):
            extensions.add("uv")

        # Check for conda environment files
        if (workspace / "environment.yml").exists() or (workspace / "environment.yaml").exists():
            extensions.add("conda")

        # Check for C/C++ files (for ccache)
        cpp_patterns = ["*.cpp", "*.hpp", "*.cc", "*.cxx", "*.h", "*.c", "*.hxx"]
        for pattern in cpp_patterns:
            if list(workspace.rglob(pattern)):
                extensions.add("ccache")
                break  # Found at least one C++ file, no need to continue

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
