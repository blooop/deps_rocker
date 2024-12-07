import pkgutil
from typing import Set
from rocker.extensions import RockerExtension
from deps_rocker.simple_rocker_extension import SimpleRockerExtension

# to run:
# isaacsim omni.isaac.sim.python.kit


class IsaacSim(SimpleRockerExtension):
    name = "isaacsim"

    def required(self, cliargs) -> Set[str]:
        return set(["nvidia", "privileged", "x11"])

    def get_docker_args(self, cliargs):
        return " --runtime=nvidia"

    @staticmethod
    def register_arguments(parser, defaults=None):
        SimpleRockerExtension.register_arguments_helper(IsaacSim.name, parser, defaults)
