from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class VcsTool(SimpleRockerExtension):
    name = "curl"

    @staticmethod
    def register_arguments(parser, defaults=None):
        SimpleRockerExtension.register_arguments_helper(VcsTool.name, parser, defaults)
