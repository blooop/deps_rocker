from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Pixi(SimpleRockerExtension):
    """Install pixi and enable shell completion"""

    name = "pixi"

    def required(self, cliargs) -> set[str]:
        # Requires curl to fetch the installer, and user to modify ~/.bashrc
        return {"curl", "user"}

    def invoke_after(self, cliargs) -> set[str]:
        # Ensure this runs after user extension is applied
        return {"user"}
