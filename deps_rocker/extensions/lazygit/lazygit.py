from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Lazygit(SimpleRockerExtension):
    """Install lazygit for interactive git operations"""

    name = "lazygit"
    depends_on_extension = ("curl", "git", "git_clone")

    def get_template_args(self, cliargs=None):
        return {
            "lazygit_version": cliargs.get("lazygit_version", "0.41.0") if cliargs else "0.41.0",
        }
