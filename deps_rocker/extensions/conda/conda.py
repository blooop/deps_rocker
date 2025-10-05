from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Conda(SimpleRockerExtension):
    """Install Miniconda for Python package and environment management"""

    name = "conda"
    depends_on_extension: tuple[str, ...] = ("curl", "user")

    def get_template_args(self, cliargs=None):
        return {
            "miniforge_version": cliargs.get("miniforge_version", "latest")
            if cliargs
            else "latest",
            "conda_version": cliargs.get("conda_version", "24.3.0-0") if cliargs else "24.3.0-0",
        }
