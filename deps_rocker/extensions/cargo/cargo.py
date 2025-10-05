from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Cargo(SimpleRockerExtension):
    """Add Rust and Cargo package manager to your docker image"""

    name = "cargo"
    depends_on_extension = ("curl",)

    # Template arguments for both snippets
    empty_args = {
        "CARGO_VERSION": "1.77.2",
    }
