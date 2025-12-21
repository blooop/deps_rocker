from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class GitUI(SimpleRockerExtension):
    """Install GitUI terminal git client via pixi global"""

    name = "gitui"
    depends_on_extension = ("pixi", "git", "git_clone")
    pixi_global_packages = [
        {
            "name": "gitui",
            "channels": ["conda-forge"],
            "deps": {"gitui": "*"},
            "exposed": {"gitui": "gitui"},
        }
    ]
