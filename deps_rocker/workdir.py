from deps_rocker.simple_rocker_extension import SimpleRockerExtension

class Workdir(SimpleRockerExtension):
    """Set the working directory inside the container"""

    name = "workdir"

    def get_docker_args(self, cliargs) -> str:
        workdir = cliargs.get("workdir", "/workspaces")
        return f"-w {workdir}" if workdir else ""

    @staticmethod
    def register_arguments(parser, defaults=None):
        parser.add_argument(
            "--workdir",
            type=str,
            default="/workspaces",
            help="Set the working directory inside the container (default: /workspaces).",
        )
