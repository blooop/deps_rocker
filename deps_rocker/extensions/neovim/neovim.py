from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class NeoVim(SimpleRockerExtension):
    """Install the latest stable Neovim and mount host configs (~/.config/nvim, ~/.vim)"""

    name = "neovim"
    depends_on_extension = ("curl",)

    def get_docker_args(self, cliargs):
        from pathlib import Path

        home = str(Path.home())
        nvim_config = f"{home}/.config/nvim"
        vim_config = f"{home}/.vim"
        args = f" -v {nvim_config}:/root/.config/nvim -v {vim_config}:/root/.vim "
        return args
