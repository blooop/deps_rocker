from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Pixi(SimpleRockerExtension):
    """Install pixi and enable shell completion"""

    name = "pixi"
    depends_on_extension: tuple[str, ...] = ("curl", "user")
    builder_pixi_packages = ["curl", "ca-certificates"]
    builder_apt_packages: list[str] = []

    # Template arguments for both snippets
    empy_args = {
        "PIXI_VERSION": "0.60.0",
    }
