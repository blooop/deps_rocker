from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Fzf(SimpleRockerExtension):
    """Adds fzf autocomplete to your container"""

    name = "fzf"
    depends_on_extension = ["git_clone", "curl", "user"]

    def get_template_args(self, cliargs=None):
        return {
            "fzf_version": cliargs.get("fzf_version", "0.53.0") if cliargs else "0.53.0",
        }
