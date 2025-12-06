from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class DepsDevtools(SimpleRockerExtension):
    """Install ripgrep, fd-find, and fzf for developer productivity via pixi."""

    name = "deps-devtools"
    depends_on_extension = ("pixi", "fzf")
