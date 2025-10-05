from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Fzf(SimpleRockerExtension):
    """Adds fzf autocomplete to your container"""

    name = "fzf"
    depends_on_extension = ["git_clone", "curl", "user"]

    # Template arguments for both snippets
    empy_args = {
        "FZF_VERSION": "0.53.0",
    }
