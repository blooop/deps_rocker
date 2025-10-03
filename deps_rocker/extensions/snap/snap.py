from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Snap(SimpleRockerExtension):
    """Install snapd package manager for snap packages"""

    name = "snap"
