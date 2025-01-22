from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class TzData(SimpleRockerExtension):
    """Sets up tzdata without requiring user input"""

    name = "tzdata"
