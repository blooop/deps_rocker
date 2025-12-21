from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Npm(SimpleRockerExtension):
    """Install npm using nodejs from conda-forge via pixi (builder only)"""

    name = "npm"
    depends_on_extension = ("user",)
