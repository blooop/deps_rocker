from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Cargo(SimpleRockerExtension):
    """Add Rust and Cargo package manager via the pixi tools environment"""

    name = "cargo"
    depends_on_extension = ("pixi",)
    pixi_dependencies = {"rust": "*"}
