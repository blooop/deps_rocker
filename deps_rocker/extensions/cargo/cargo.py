from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Cargo(SimpleRockerExtension):
    """Add Rust and Cargo package manager to your docker image"""

    name = "cargo"
    depends_on_extension = ("curl",)

    # Template arguments for both snippets
    empy_args = {
        "cargo_version": "1.77.2",
    }

    empy_builder_args = {
        "cargo_version": "1.77.2",
    }
