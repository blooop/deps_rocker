from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class GitClone(SimpleRockerExtension):
    """Adds support for git cloning via pixi"""

    name = "git_clone"
    depends_on_extension = ["pixi", "git"]
    pixi_global_packages = [
        {
            "name": "git",
            "channels": ["conda-forge"],
            "deps": {"git": "*"},
            "exposed": {
                "git": "git",
                "git-cvsserver": "git-cvsserver",
                "git-receive-pack": "git-receive-pack",
                "git-shell": "git-shell",
                "git-upload-archive": "git-upload-archive",
                "git-upload-pack": "git-upload-pack",
                "gitk": "gitk",
                "scalar": "scalar",
            },
        },
        {
            "name": "git-lfs",
            "channels": ["conda-forge"],
            "deps": {"git-lfs": "*"},
            "exposed": {"git-lfs": "git-lfs"},
        },
    ]
