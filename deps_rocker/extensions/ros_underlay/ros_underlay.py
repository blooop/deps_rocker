from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension
from deps_rocker.extensions.repository_discovery_mixin import RepositoryDiscoveryMixin


class RosUnderlay(RepositoryDiscoveryMixin, SimpleRockerExtension):
    """Build ROS underlay from vcstool repositories"""

    name = "ros_underlay"
    depends_on_extension = ("vcstool", "ros_jazzy")

    def __init__(self) -> None:
        super().__init__()
        self.discover_repos()

    def get_files(self, cliargs) -> dict:
        """Copy build-underlay script to Docker context"""
        script_path = Path(__file__).parent / "build-underlay.sh"
        with script_path.open(encoding="utf-8") as f:
            return {"ros_underlay/build-underlay.sh": f.read()}
