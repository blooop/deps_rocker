from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Curl(SimpleRockerExtension):
    name = "curl"

    @staticmethod
    def register_arguments(parser, defaults=None):
        SimpleRockerExtension.register_arguments_helper(Curl.name, parser, defaults)
