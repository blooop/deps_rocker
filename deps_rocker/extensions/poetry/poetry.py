from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Poetry(SimpleRockerExtension):
    """Install Poetry for Python dependency management"""

    name = "poetry"
    depends_on_extension: tuple[str, ...] = ("curl",)
    builder_apt_packages = ["curl", "ca-certificates", "python3"]
    apt_packages = ["python3", "python3-venv"]

    # Template arguments for both snippets
    empy_args = {
        "POETRY_VERSION": "1.8.3",
    }
