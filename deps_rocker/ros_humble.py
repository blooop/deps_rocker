import pkgutil
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class RosHumble(SimpleRockerExtension):
    name = "ros_humble"

    @staticmethod
    def register_arguments(parser, defaults=None):
        return SimpleRockerExtension.register_arguments_helper(RosHumble.name, parser, defaults)
