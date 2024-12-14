from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class CWD(SimpleRockerExtension):
    """Add the current working directory as a volume in your docker container"""

    name = "cwd"

    def get_docker_args(self, cliargs) -> str:
        return " -v %s:%s " % (Path.cwd(), "/workspaces")

    def invoke_after(self, cliargs) -> set:
        # return set(["vcstool"])
        return {"user"}

    @staticmethod
    def register_arguments(parser, defaults=None) -> None:
        SimpleRockerExtension.register_arguments_helper(CWD, parser, defaults)

class CWDName(SimpleRockerExtension):
    """Set the name of the container to the name of the folder of the current working directory"""

    name = "cwd_name"

    def get_docker_args(self, cliargs) -> str:
        return f" --name {Path.cwd().stem}"

    @staticmethod
    def register_arguments(parser, defaults=None) -> None:
        SimpleRockerExtension.register_arguments_helper(CWDName, parser, defaults)
