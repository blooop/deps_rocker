from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Jquery(SimpleRockerExtension):
    """Installs the `jq` JSON processor via pixi (extension named jquery per request)."""

    name = "jquery"
    depends_on_extension = ("pixi",)
