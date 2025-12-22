from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class GitUI(SimpleRockerExtension):
    """Install GitUI terminal git client via the pixi tools environment"""

    name = "gitui"
    depends_on_extension = ("pixi",)
    pixi_dependencies = {"gitui": "*"}
