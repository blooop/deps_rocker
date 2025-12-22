from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Npm(SimpleRockerExtension):
    """Install npm using nvm (Node Version Manager)"""

    name = "npm"
    depends_on_extension = ("curl",)
    builder_apt_packages = ["curl", "ca-certificates", "git"]

    empy_args = {
        "NODE_VERSION": "24.9.0",
        "NPM_VERSION": "11.6.1",
    }
