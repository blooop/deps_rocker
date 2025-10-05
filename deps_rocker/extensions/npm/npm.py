from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Npm(SimpleRockerExtension):
    """Install npm using nvm (Node Version Manager)"""

    name = "npm"
    depends_on_extension = ("curl",)

    empy_args = {
        "node_version": "24.9.0",
        "npm_version": "11.6.1",
    }

