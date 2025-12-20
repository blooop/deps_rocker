from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Pixi(SimpleRockerExtension):
    """Install pixi and enable shell completion"""

    name = "pixi"
    depends_on_extension: tuple[str, ...] = ("user",)
    builder_apt_packages = ["curl", "ca-certificates"]

    # Template arguments for both snippets
    empy_args = {
        "PIXI_VERSION": "0.62.1",
    }
