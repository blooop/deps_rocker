from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Curl(SimpleRockerExtension):
    """Adds curl to your docker container"""

    name = "curl"
    depends_on_extension = ("apt_update",)
    apt_packages = ["curl", "ca-certificates"]
