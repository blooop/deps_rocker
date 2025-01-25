from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Locales(SimpleRockerExtension):
    """Sets up locales in your docker container. Defaults to US locale"""

    name = "locales"
