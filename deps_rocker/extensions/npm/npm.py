from deps_rocker.simple_rocker_extension import SimpleRockerExtension





class Npm(SimpleRockerExtension):
    """Install npm using nvm (Node Version Manager)"""

    name = "npm"
    depends_on_extension = ("curl",)

    empy_args = {
        "node_version": "24.9.0",
        "npm_version": "11.6.1",
        "builder_output_dir": "/opt/deps_rocker/npm",
        "builder_stage": "npm_builder",
    }

    empy_builder_args = {
        "node_version": "24.9.0",
        "builder_output_dir": "/opt/deps_rocker/npm",
        "builder_stage": "npm_builder",
    }
