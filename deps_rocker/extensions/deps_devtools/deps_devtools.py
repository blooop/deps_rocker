from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class DepsDevtools(SimpleRockerExtension):
    """Install ripgrep, fd-find, and fzf for developer productivity."""

    name = "deps-devtools"
    depends_on_extension = ("fzf",)

    apt_packages = ["ripgrep", "fd-find"]
