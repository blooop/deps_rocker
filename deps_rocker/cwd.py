from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class CWD(SimpleRockerExtension):
    """Add the current working directory as a volume in your docker container"""

    name = "cwd"

    def get_docker_args(self, cliargs):
        return " -v %s:%s " % (Path.cwd(), "/workspaces")

    # def invoke_after(self, cliargs):
    #     return set(["vcstool"])

    @staticmethod
    def register_arguments(parser, defaults=None):
        SimpleRockerExtension.register_arguments_helper(CWD.name, parser, defaults)
