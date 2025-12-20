import warnings
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class IsaacSim(SimpleRockerExtension):
    """DEPRECATED: Isaac Sim extension has been removed due to disk space constraints in CI.

    This extension is no longer maintained and will be removed in a future version.
    Please use the official Isaac Sim Docker images directly from NVIDIA instead.
    """

    name = "isaacsim"

    def __init__(self):
        super().__init__()
        warnings.warn(
            "The isaac_sim extension is deprecated and will be removed in a future version. "
            "It has been removed due to disk space constraints in CI. "
            "Please use the official Isaac Sim Docker images directly from NVIDIA instead.",
            DeprecationWarning,
            stacklevel=2,
        )

    def get_snippet(self, cliargs):
        # Return empty snippet - extension does nothing
        return ""
