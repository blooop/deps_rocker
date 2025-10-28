from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Nvim(SimpleRockerExtension):
    """Install nvim v0.11.4 and mount host configs (~/.config/nvim, ~/.vim)"""

    name = "nvim"
    depends_on_extension = ("curl",)
    builder_apt_packages = ["curl", "ca-certificates"]

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
