from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class AptUpdate(SimpleRockerExtension):
    """Update apt package lists for subsequent package installations"""

    name = "apt_update"