"""Mixin class for repository discovery functionality"""

from pathlib import Path
import itertools


class RepositoryDiscoveryMixin:
    """Mixin class providing common repository discovery functionality
    
    This mixin provides methods for discovering *.repos and depends.repos.yaml files
    and is intended to be used by extensions that need to work with vcstool repositories.
    """

    def discover_repos(self):
        """Discover all *.repos and depends.repos.yaml files recursively
        
        Populates self.empy_args["depend_repos"] with discovered repository files.
        Each entry contains 'dep' (relative path) and 'path' (parent directory).
        """
        if not hasattr(self, 'empy_args'):
            self.empy_args = {}
        
        if "depend_repos" not in self.empy_args:
            self.empy_args["depend_repos"] = []

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

    def get_repo_files_content(self) -> dict:
        """Get content of all discovered repository files
        
        Returns:
            dict: Dictionary mapping relative file paths to their content
        """
        files_content = {}
        
        if hasattr(self, 'empy_args') and "depend_repos" in self.empy_args:
            for repo_info in self.empy_args["depend_repos"]:
                repo_path = Path.cwd() / repo_info["dep"]
                if repo_path.is_file():
                    with repo_path.open(encoding="utf-8") as f:
                        files_content[repo_info["dep"]] = f.read()
        
        return files_content