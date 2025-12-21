from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Lazygit(SimpleRockerExtension):
    """Install lazygit for interactive git operations via pixi"""

    name = "lazygit"
    depends_on_extension = ("pixi", "git")
    pixi_global_packages = [
        {
            "name": "lazygit",
            "channels": ["conda-forge"],
            "deps": {"lazygit": "*"},
            "exposed": {"lazygit": "lazygit"},
        }
    ]
