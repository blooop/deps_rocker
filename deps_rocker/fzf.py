from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Fzf(SimpleRockerExtension):
    """Adds fzf to your container"""

    name = "fzf"

    def get_user_snippet(self, cliargs):
        return """
        # Install fzf from source as apt is very out of date
        git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf;  ~/.fzf/install --all
        """
    
    def invoke_after(self, cliargs) -> set:
        return {"user"}
