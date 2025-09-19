from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Lazygit(SimpleRockerExtension):
    """Install lazygit binary from latest GitHub release"""

    name = "lazygit"

    def required(self, cliargs) -> set[str]:
        # Needs curl to fetch release info and git as a common dependency
        return {"curl", "git"}
