from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class UV(SimpleRockerExtension):
    """Add the uv package manager to your docker image"""

    name = "uv"
