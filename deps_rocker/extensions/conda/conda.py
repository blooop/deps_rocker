from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Conda(SimpleRockerExtension):
    """Install Miniforge (conda) and shell activation hooks"""

    name = "conda"

    def required(self, cliargs) -> set[str]:
        # Needs curl to download installer; user to update ~/.bashrc
        return {"curl", "user"}

    def invoke_after(self, cliargs) -> set[str]:
        return {"user"}
