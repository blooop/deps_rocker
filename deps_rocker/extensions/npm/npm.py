from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Npm(SimpleRockerExtension):
    """Install npm using nvm (Node Version Manager)"""

    name = "npm"
    depends_on_extension = ("curl",)

    # Template arguments for the main snippet
    empy_args = {
        "node_version": "24.9.0",
        "npm_version": "11.6.1",
        "nvm_version": "0.40.0",
    }

    # Template arguments for the builder snippet
    empy_builder_args = {
        "node_version": "24.9.0",
        "npm_version": "11.6.1",
        "nvm_version": "0.40.0",
    }
