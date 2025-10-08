from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Neovim(SimpleRockerExtension):
    """Install the latest stable Neovim and mount host configs (~/.config/nvim, ~/.vim)"""

    name = "neovim"
    depends_on_extension = ("curl",)

    def required(self, container):
        return [
            {
                "type": "mount",
                "source": "~/.config/nvim",
                "target": "/root/.config/nvim",
                "readonly": False,
            },
            {"type": "mount", "source": "~/.vim", "target": "/root/.vim", "readonly": False},
        ]
