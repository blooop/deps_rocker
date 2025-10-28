from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Lazygit(SimpleRockerExtension):
    """Install lazygit for interactive git operations"""

    name = "lazygit"
    depends_on_extension = ("curl", "git", "git_clone")
    builder_apt_packages = ["curl", "ca-certificates", "tar"]

    # Template arguments for both snippets
    empy_args = {
        "LAZYGIT_VERSION": "0.41.0",
    }
