from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Fzf(SimpleRockerExtension):
    """Adds fzf autocomplete to your container"""

    name = "fzf"

    def invoke_after(self, cliargs) -> set:
        return {"user"}

    def required(self, cliargs):
        return {"git","git_clone","curl"}