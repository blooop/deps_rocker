from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class UV(SimpleRockerExtension):
    """Add the uv package manager to your docker image"""

    name = "uv"

    def required(self, cliargs):
        return {"curl"}

    def invoke_after(self, cliargs):
        return {"curl"}

    # TODO enable use of cache on host machine
    # def get_docker_args(self, cliargs):
    #     # abs = Path("$HOME/.cache/uv").absolute().as_posix()
    #     abs = "$HOME/.cache/uv"
    #     return f"-v {abs}:{abs}"
