from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Pixi(SimpleRockerExtension):
    """Install pixi and enable shell completion"""

    name = "pixi"
    depends_on_extension: tuple[str, ...] = ("curl", "user")

    def get_template_args(self, cliargs=None):
        return {
            "pixi_version": cliargs.get("pixi_version", "0.21.0") if cliargs else "0.21.0",
        }
