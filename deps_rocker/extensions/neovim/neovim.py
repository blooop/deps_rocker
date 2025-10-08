from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class NeoVim(SimpleRockerExtension):
    """Install the latest stable Neovim and mount host configs (~/.config/nvim, ~/.vim)"""

    name = "neovim"
    depends_on_extension = ("curl",)

    # Template arguments for both snippets
    empy_args = {
        "NEOVIM_VERSION": "v0.11.4",
    }

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
