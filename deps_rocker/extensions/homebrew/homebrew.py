from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Homebrew(SimpleRockerExtension):
    """Install Homebrew package manager"""

    name = "homebrew"
    builder_apt_packages = ["curl", "git", "file", "build-essential", "procps"]
    depends_on_extension = (
        "curl",
        "git",
    )
