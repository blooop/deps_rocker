from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Detach(SimpleRockerExtension):
    """Run the container in detached mode (in the background)"""

    name = "detach"

    def get_docker_args(self, _cliargs) -> str:
        return " --detach"
