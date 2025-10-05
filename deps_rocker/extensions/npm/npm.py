from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Npm(SimpleRockerExtension):
    """Install npm using nvm (Node Version Manager)"""

    name = "npm"
    depends_on_extension = ("curl",)

    def get_template_args(self, cliargs=None):
        # Provide default versions if not specified
        return {
            "node_version": cliargs.get("node_version", "24.9.0") if cliargs else "24.9.0",
            "npm_version": cliargs.get("npm_version", "11.6.1") if cliargs else "11.6.1",
            "nvm_version": cliargs.get("nvm_version", "0.40.0") if cliargs else "0.40.0",
        }
