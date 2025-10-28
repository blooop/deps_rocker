from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Conda(SimpleRockerExtension):
    """Install Miniconda for Python package and environment management"""

    name = "conda"
    depends_on_extension: tuple[str, ...] = ("curl", "user")
    builder_apt_packages = ["curl", "ca-certificates", "bzip2"]

    # Template arguments for both snippets
    empy_args = {
        "MINIFORGE_VERSION": "latest",
        "CONDA_VERSION": "24.3.0-0",
    }
