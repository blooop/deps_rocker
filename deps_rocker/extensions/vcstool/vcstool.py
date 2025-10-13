from __future__ import annotations


import yaml
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class VcsTool(SimpleRockerExtension):
    """Add vcstool to the container and clones any repos found in depends.repos files"""

    name = "vcstool"
    apt_packages = ["python3-pip", "git", "git-lfs"]

    def get_files(self, cliargs) -> dict:
        """Discover and merge all depends.repos files into a single consolidated manifest

        Returns:
            dict: Single consolidated.repos file containing all discovered repositories
        """
        workspace = self.get_workspace_path()

        workspace = Path(cliargs.get("auto", workspace)).expanduser()
        print(cliargs)

        print("vsc tool search root:", workspace)
        merged_repos = {"repositories": {}}

        # Search only for files named exactly "depends.repos"
        for repos_file in workspace.rglob("depends.repos*"):
            print("found repos file:", repos_file)
            if repos_file.is_file():
                with repos_file.open(encoding="utf-8") as f:
                    repos_data = yaml.safe_load(f)
                    if repos_data and "repositories" in repos_data:
                        # Merge repositories from this file into the consolidated manifest
                        merged_repos["repositories"].update(repos_data["repositories"])

        # Always return consolidated.repos file, even if empty
        return {"consolidated.repos": yaml.dump(merged_repos, default_flow_style=False)}
