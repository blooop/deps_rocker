from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class UV(SimpleRockerExtension):
    name = "uv"

    def required(self, cliargs):
        return set(["curl"])

    def invoke_after(self, cliargs):
        return set(["curl"])

    @staticmethod
    def register_arguments(parser, defaults=None):
        SimpleRockerExtension.register_arguments_helper(UV.name, parser, defaults)
