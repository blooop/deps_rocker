from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Cargo(SimpleRockerExtension):
    """Add Rust and Cargo package manager to your docker image"""

    name = "cargo"
    depends_on_extension = ("curl",)

    def get_template_args(self, cliargs=None):
        return {
            "cargo_version": cliargs.get("cargo_version", "1.77.2") if cliargs else "1.77.2",
        }
