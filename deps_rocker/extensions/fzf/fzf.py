from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Fzf(SimpleRockerExtension):
    """Adds fzf autocomplete to your container"""

    name = "fzf"
    depends_on_extension = ["git_clone", "curl", "user"]
