from deps_rocker.simple_rocker_extension import SimpleRockerExtension
import hashlib
import json
import os
from pathlib import Path
from typing import Dict, List


class RosGeneric(SimpleRockerExtension):
    """Adds a configurable ROS 2 distribution (default: jazzy) to your docker container"""

    name = "ros_generic"
    depends_on_extension = ("locales", "tzdata", "curl", "vcstool")

    def get_files(self, cliargs) -> dict[str, str]:
        files: Dict[str, str] = {
            "defaults.yaml": self.get_config_file("configs/defaults.yaml"),
            "ros_underlays.py": self.get_config_file("scripts/ros_underlays.py"),
        }

        manifests = self._discover_repos_manifests()
        manifest_payload = {
            "version": 1,
            "search_roots": ["/dependencies"],
            "manifests": manifests,
        }
        files["ros_underlays_manifest.json"] = json.dumps(manifest_payload, indent=2) + "\n"

        self.empy_args["has_underlay_manifests"] = bool(manifests)
        self.empy_args["ros_underlay_manifest_file"] = "ros_underlays_manifest.json"
        return files

    def get_ros_distro(self, cliargs):
        # Allow override via cliargs, else default to jazzy
        return cliargs.get("ros_distro", "jazzy")

    @property
    def empy_args(self):
        return {
            "ros_distro": "jazzy"  # default value
        }

    @property
    def empy_builder_args(self):
        return {
            "ros_distro": "jazzy"  # default value
        }

    def get_docker_args(self, cliargs) -> str:
        """Return a string of --env args for Docker run, space-separated."""
        ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID")
        if ROS_DOMAIN_ID is None:
            username = os.environ.get("USER")
            if username:
                hashed_value = int(hashlib.sha256(username.encode()).hexdigest(), 16)
                ROS_DOMAIN_ID = str((hashed_value % 99) + 1)
            else:
                raise ValueError("Unable to determine username and no ROS_DOMAIN_ID provided.")
        return (
            f" --env ROS_DOMAIN_ID={ROS_DOMAIN_ID} --env ROS_DISTRO={self.get_ros_distro(cliargs)}"
        )

    def _discover_repos_manifests(self) -> List[Dict[str, str]]:
        workspace = Path.cwd()
        manifests: List[Dict[str, str]] = []
        for path in workspace.rglob("*.repos"):
            if not path.is_file():
                continue
            if self._should_skip_repos(path):
                continue
            rel = path.relative_to(workspace)
            rel_posix = rel.as_posix()
            manifests.append(
                {
                    "id": self._make_underlay_identifier(rel_posix),
                    "manifest_path": self._container_manifest_path(rel),
                    "source_path": self._container_source_path(rel),
                    "display_name": rel_posix,
                    "relative_path": rel_posix,
                    "order": self._order_hint(rel),
                }
            )
        manifests.sort(key=lambda item: (item["order"], item["relative_path"]))
        return manifests

    def _container_manifest_path(self, rel_path: Path) -> str:
        rel_posix = rel_path.as_posix()
        if rel_posix == ".":
            return "/dependencies"
        return f"/dependencies/{rel_posix}"

    def _container_source_path(self, rel_path: Path) -> str:
        parent = rel_path.parent.as_posix()
        if parent in (".", ""):
            return "/dependencies"
        return f"/dependencies/{parent}"

    def _should_skip_repos(self, path: Path) -> bool:
        ignored = {"build", "install", "log", "logs", "__pycache__"}
        for part in path.parts:
            if part.startswith(".") and part not in {"."}:
                return True
            if part in ignored:
                return True
        return False

    def _make_underlay_identifier(self, rel_posix: str) -> str:
        key = f"/dependencies::{rel_posix}"
        digest = hashlib.sha256(key.encode("utf-8")).hexdigest()[:10]
        safe = "".join(ch if ch.isalnum() else "_" for ch in key)
        safe = "_".join(filter(None, safe.split("_")))
        safe = safe[:48] if safe else "underlay"
        return f"{safe}_{digest}"

    def _order_hint(self, rel_path: Path) -> int:
        depth = len(rel_path.parts)
        filename = rel_path.name.lower()
        depends_priority = 0 if "depends" in filename else 1
        return depends_priority * 100 + depth
