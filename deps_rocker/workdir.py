from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Workdir(SimpleRockerExtension):
    """Set the working directory inside the container"""

    name = "workdir"

    def get_docker_args(self, cliargs) -> str:
        workdir = cliargs.get("workdir", ["/workspaces"])
        # workdir will be a list if passed via nargs
        if isinstance(workdir, list):
            workdir_val = workdir[0] if workdir else "/workspaces"
        else:
            workdir_val = workdir
        return f"-w {workdir_val}" if workdir_val else ""

    @staticmethod
    def register_arguments(parser, defaults=None):
        parser.add_argument(
            "--workdir",
            metavar="PATH",
            type=str,
            nargs=1,
            default=["/workspaces"],
            help="Set the working directory inside the container (default: /workspaces).",
        )
