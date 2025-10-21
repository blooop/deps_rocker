from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Chrome(SimpleRockerExtension):
    """Install Google Chrome browser"""

    name = "chrome"
    depends_on_extension = ("curl",)
