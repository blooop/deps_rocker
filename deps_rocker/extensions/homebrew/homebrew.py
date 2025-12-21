from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Homebrew(SimpleRockerExtension):
    """Install Homebrew package manager"""

    name = "homebrew"
    builder_apt_packages = [
        "build-essential",
        "procps",
        "curl",
        "file",
        "git",
        "ca-certificates",
    ]
