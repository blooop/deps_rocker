from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class NeoVim(SimpleRockerExtension):
    """Add neovim to your docker image"""

    name = "neovim"
    depends_on_extension = ("apt_update",)
    apt_packages = ["neovim"]
