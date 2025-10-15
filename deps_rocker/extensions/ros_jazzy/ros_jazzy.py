import os
import hashlib
import yaml
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class RosJazzy(SimpleRockerExtension):
    """Adds ros-jazzy to your docker container"""

    name = "ros_jazzy"

    depends_on_extension = ("curl", "git_clone")
    # Use apt_packages feature for ROS dependencies
    apt_packages = [
        "locales",
        "tzdata",
        "curl",
        "gnupg2",
        "lsb-release",
        "sudo",
        "software-properties-common",
        "wget",
        "python3-pip",
        "cmake",
        "build-essential",
        "python3-argcomplete",
    ]

    def invoke_after(self, cliargs):
        return super().invoke_after({"gemini", "claude", "codex"})

    def get_files(self, cliargs) -> dict[str, str]:
        dat = self.get_config_file("configs/colcon-defaults.yaml")

        # Get underlay build scripts
        script_dir = Path(__file__).parent
        underlay_deps = (script_dir / "underlay_deps.sh").read_text()
        underlay_build = (script_dir / "underlay_build.sh").read_text()

        # Discover and merge all depends.repos files
        workspace = self.get_workspace_path()
        workspace = Path(cliargs.get("auto", workspace)).expanduser()

        print("ROS Jazzy: searching for depends.repos in:", workspace)
        merged_repos = {"repositories": {}}

        # Check for test depends.repos in extension directory (for testing)
        test_depends_file = script_dir / "test_depends.repos"
        if test_depends_file.is_file():
            print("ROS Jazzy: found test depends file:", test_depends_file)
            with test_depends_file.open(encoding="utf-8") as f:
                repos_data = yaml.safe_load(f)
                if repos_data and "repositories" in repos_data:
                    merged_repos["repositories"].update(repos_data["repositories"])

        # Search for files named "depends.repos" in workspace
        for repos_file in workspace.rglob("depends.repos*"):
            if repos_file.is_file():
                print("ROS Jazzy: found repos file:", repos_file)
                with repos_file.open(encoding="utf-8") as f:
                    repos_data = yaml.safe_load(f)
                    if repos_data and "repositories" in repos_data:
                        # Merge repositories from this file into the consolidated manifest
                        merged_repos["repositories"].update(repos_data["repositories"])

        print("ROS Jazzy: merged repos:", merged_repos)

        # Include test files for the test script
        test_package_xml = (script_dir / "test_package.xml").read_text()
        test_setup_py = (script_dir / "setup_py_template").read_text()

        return {
            "colcon-defaults.yaml": dat,
            "underlay_deps.sh": underlay_deps,
            "underlay_build.sh": underlay_build,
            "consolidated.repos": yaml.dump(merged_repos, default_flow_style=False),
            "test_package.xml": test_package_xml,
            "test_setup.py": test_setup_py,
        }

    def get_docker_args(self, cliargs) -> str:
        """Set the ROS_DOMAIN_ID env var from the host machine if it exists, otherwise generate one from a hash of the username"""
        ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID")
        if ROS_DOMAIN_ID is None:
            username = os.environ.get("USER")
            if username:
                # Hash the username
                hashed_value = int(hashlib.sha256(username.encode()).hexdigest(), 16)
                # Scale the hash to a value between 2 and 99
                ROS_DOMAIN_ID = str((hashed_value % 98) + 2)
            else:
                raise ValueError("Unable to determine username and no ROS_DOMAIN_ID provided.")

        return f" --env ROS_DOMAIN_ID={ROS_DOMAIN_ID}"
