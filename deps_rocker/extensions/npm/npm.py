from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Npm(SimpleRockerExtension):
    """Install npm via pixi using nodejs from conda-forge"""

    name = "npm"
    depends_on_extension = ("pixi",)
