from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Lazygit(SimpleRockerExtension):
    """Install lazygit for interactive git operations"""

    name = "lazygit"
    depends_on_extension = ("curl", "git", "git_clone")

    # Template arguments for both snippets
    empy_args = {
        "lazygit_version": "0.41.0",
    }

    empy_builder_args = {
        "lazygit_version": "0.41.0",
    }
