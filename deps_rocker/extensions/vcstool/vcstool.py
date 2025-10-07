from deps_rocker.simple_rocker_extension import SimpleRockerExtension
from deps_rocker.extensions.repository_discovery_mixin import RepositoryDiscoveryMixin


class VcsTool(RepositoryDiscoveryMixin, SimpleRockerExtension):
    """Add vcstool to the container and clones any repos found in *.repos files"""

    name = "vcstool"
    apt_packages = ["python3-pip", "git", "git-lfs"]

    def __init__(self) -> None:
        super().__init__()
        self.discover_repos()

    def get_files(self, cliargs) -> dict:
        return self.get_repo_files_content()

    # def invoke_after(self, cliargs):
    #     return set(["cwd", "user"])
