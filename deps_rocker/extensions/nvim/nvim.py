from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Nvim(SimpleRockerExtension):
    """Install Neovim from the shared pixi tools env and mount host configs (~/.config/nvim, ~/.vim)"""

    name = "nvim"
    depends_on_extension = ("pixi",)
    pixi_channels = ("https://prefix.dev/github-releases", "conda-forge")
    pixi_dependencies = {"nvim": "*"}

    def get_docker_args(self, cliargs):
        from pathlib import Path

        args = ""
        home = Path.home()

        nvim_config = home / ".config" / "nvim"
        if nvim_config.exists():
            args += f" -v {nvim_config}:/root/.config/nvim"

        vim_config = home / ".vim"
        if vim_config.exists():
            args += f" -v {vim_config}:/root/.vim"

        return args
