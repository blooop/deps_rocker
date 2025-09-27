from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Lazygit(SimpleRockerExtension):
    """Install lazygit for interactive git operations"""

    name = "lazygit"
    depends_on_extension = ("curl", "git_clone")
