from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class UV(SimpleRockerExtension):
    name = "uv"

    def required(self, cliargs):
        return {"curl"}

    def invoke_after(self, cliargs):
        return set(["curl"])
    
    #TODO enable use of cache on host machine
    # def get_docker_args(self, cliargs):
    #     # abs = Path("$HOME/.cache/uv").absolute().as_posix()
    #     abs = "$HOME/.cache/uv"
    #     return f"-v {abs}:{abs}"

