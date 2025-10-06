from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class RosUnderlay(SimpleRockerExtension):
    """Build ROS underlay from vcstool repositories"""

    name = "ros_underlay"
    depends_on_extension = ("vcstool", "ros_jazzy")

    def get_files(self, cliargs) -> dict:
        """Copy build-underlay script to Docker context"""
        script_path = Path(__file__).parent / "build-underlay.sh"
        with script_path.open(encoding="utf-8") as f:
            return {"ros_underlay/build-underlay.sh": f.read()}
