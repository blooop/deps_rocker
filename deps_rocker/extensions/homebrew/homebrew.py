from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Homebrew(SimpleRockerExtension):
    """Install Homebrew package manager"""

    name = "homebrew"
    depends_on_extension = ("curl",)
