from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Npm(SimpleRockerExtension):
    """Install npm using nodejs from conda-forge via the shared pixi tools environment"""

    name = "npm"
    depends_on_extension = ("pixi", "user")
    pixi_dependencies = {"nodejs": ">=20,<21"}
