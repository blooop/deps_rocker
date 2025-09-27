from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class GitClone(SimpleRockerExtension):
    """Adds support for git cloning"""

    name = "git_clone"
    depends_on_extension = ("apt_update", "git")
    apt_packages = ["git", "git-lfs", "ca-certificates"]
