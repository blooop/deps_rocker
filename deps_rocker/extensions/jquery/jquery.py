from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Jquery(SimpleRockerExtension):
    """Installs the `jq` JSON processor via apt (extension named jquery per request)."""

    name = "jquery"
    apt_packages = ["jq"]
