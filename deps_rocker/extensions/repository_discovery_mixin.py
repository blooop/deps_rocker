"""Mixin class for repository discovery functionality"""

from __future__ import annotations

from pathlib import Path
import itertools


DEFAULT_WORKSPACE_ROOT = Path("/workspaces/ros_ws")


class RepositoryDiscoveryMixin:
    """Mixin class providing common repository discovery functionality

    This mixin provides methods for discovering *.repos and depends.repos.yaml files
    and is intended to be used by extensions that need to work with vcstool repositories.
    """

    workspace_root: Path = DEFAULT_WORKSPACE_ROOT

    def _ensure_empy_args(self) -> None:
        """Ensure empy_args exists and contains the workspace layout defaults."""

        if not hasattr(self, "empy_args"):
            self.empy_args = {}

        layout = self._get_workspace_layout()
        self.workspace_layout = layout

        for key, value in layout.items():
            self.empy_args.setdefault(key, value)

        self.empy_args.setdefault("depend_repos", [])

    def _get_workspace_layout(self) -> dict[str, str]:
        """Return the canonical workspace layout as POSIX strings."""

        root = Path(getattr(self, "workspace_root", DEFAULT_WORKSPACE_ROOT))
        root = root if root.is_absolute() else (DEFAULT_WORKSPACE_ROOT / root)

        return {
            "workspace_root": root.as_posix(),
            "repos_root": (root / "repos").as_posix(),
            "dependencies_root": (root / "src").as_posix(),
            "underlay_path": (root / "underlay").as_posix(),
        }

    def discover_repos(self):
        """Discover all *.repos and depends.repos.yaml files recursively

        Populates self.empy_args["depend_repos"] with discovered repository files.
        Each entry contains 'dep' (relative path) and 'path' (parent directory).
        """

        self._ensure_empy_args()

        # Get workspace path using centralized method
        workspace = self.get_workspace_path()

        # Search for both *.repos and depends.repos.yaml files
        repos_patterns = [
            workspace.rglob("*.repos"),
            workspace.rglob("depends.repos.yaml"),
        ]

        for r in itertools.chain(*repos_patterns):
            if r.is_file():
                rel_path = r.relative_to(workspace).as_posix()
                self.empy_args["depend_repos"].append(
                    dict(dep=rel_path, path=Path(rel_path).parent.as_posix())
                )

    def get_repo_files_content(self) -> dict:
        """Get content of all discovered repository files

        Returns:
            dict: Dictionary mapping relative file paths to their content
        """
        files_content = {}

        if hasattr(self, "empy_args") and "depend_repos" in self.empy_args:
            workspace = self.get_workspace_path()
            for repo_info in self.empy_args["depend_repos"]:
                repo_path = workspace / repo_info["dep"]
                if repo_path.is_file():
                    with repo_path.open(encoding="utf-8") as f:
                        files_content[repo_info["dep"]] = f.read()

        return files_content
