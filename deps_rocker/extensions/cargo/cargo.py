from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Cargo(SimpleRockerExtension):
    """Add Rust and Cargo package manager to your docker image"""

    name = "cargo"
    depends_on_extension = ("curl",)
