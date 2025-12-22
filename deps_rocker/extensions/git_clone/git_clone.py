from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class GitClone(SimpleRockerExtension):
    """Adds support for git cloning"""

    name = "git_clone"
    depends_on_extension = ("pixi",)
    pixi_dependencies = {"git": "*", "git-lfs": "*", "ca-certificates": "*"}
