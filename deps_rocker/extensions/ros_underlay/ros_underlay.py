from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class RosUnderlay(SimpleRockerExtension):
    """Build ROS underlay from vcstool repositories"""

    name = "ros_underlay"
    depends_on_extension = ("vcstool",)

    def __init__(self) -> None:
        self.empy_args["depend_repos"] = []
        self.discover_repos()

    def discover_repos(self):
        """Discover all *.repos and depends.repos.yaml files recursively"""
        import itertools

        # Search for both *.repos and depends.repos.yaml files
        repos_patterns = [
            Path.cwd().rglob("*.repos"),
            Path.cwd().rglob("depends.repos.yaml"),
        ]

        for r in itertools.chain(*repos_patterns):
            if r.is_file():
                rel_path = r.relative_to(Path.cwd()).as_posix()
                self.empy_args["depend_repos"].append(
                    dict(dep=rel_path, path=Path(rel_path).parent.as_posix())
                )

    def get_files(self, cliargs) -> dict:
        """Copy build-underlay script to Docker context"""
        script_path = Path(__file__).parent / "build-underlay.sh"
        with script_path.open(encoding="utf-8") as f:
            return {"ros_underlay/build-underlay.sh": f.read()}
