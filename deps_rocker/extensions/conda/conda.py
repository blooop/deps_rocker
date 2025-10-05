from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Conda(SimpleRockerExtension):
    """Install Miniconda for Python package and environment management"""

    name = "conda"
    depends_on_extension: tuple[str, ...] = ("curl", "user")

    # Template arguments for both snippets
    empy_args = {
        "miniforge_version": "latest",
        "conda_version": "24.3.0-0",
    }
