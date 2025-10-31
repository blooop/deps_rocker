import os
import hashlib
import yaml
from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class RosJazzy(SimpleRockerExtension):
    """Adds ros-jazzy to your docker container"""

    name = "ros_jazzy"

    depends_on_extension = ("curl", "git_clone", "user")
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
        "python3-dev",  # Required for Python development headers
        "python3-numpy",  # Required for rosidl_generator_py when building ROS packages
        "cmake",
        "build-essential",
        "python3-argcomplete",
    ]

    def invoke_after(self, cliargs):
        return super().invoke_after({"gemini", "claude", "codex", "user"})

    def _build_template_args(self, cliargs, empy_args=None) -> dict:
        """Override to add username to template context"""
        import pwd

        args = super()._build_template_args(cliargs, empy_args)
        
        # Get the actual username - this is the user that will be created in the container
        # The rocker user extension uses the current host username by default
        try:
            username = pwd.getpwuid(os.getuid()).pw_name
        except (OSError, KeyError):
            # Fallback to environment variable
            username = os.getenv('USER', 'user')

        args["name"] = username
        return args

    def get_files(self, cliargs) -> dict[str, str]:
        dat = self.get_config_file("configs/colcon-defaults.yaml")

        # Get underlay build scripts and rosdeps installer
        script_dir = Path(__file__).parent
        underlay_deps = (script_dir / "underlay_deps.sh").read_text()
        underlay_build = (script_dir / "underlay_build.sh").read_text()
        install_rosdeps = (script_dir / "install_rosdeps.sh").read_text()

        # Get unified workspace architecture scripts
        rosdep_underlay = (script_dir / "rosdep_underlay.sh").read_text()
        rosdep_overlay = (script_dir / "rosdep_overlay.sh").read_text()
        build_underlay = (script_dir / "build_underlay.sh").read_text()
        update_repos = (script_dir / "update_repos.sh").read_text()

        # Discover and merge all depends.repos files
        workspace = self.get_workspace_path()
        auto_value = cliargs.get("auto", workspace)
        # Handle case where auto is True (from CLI flag) vs actual path
        if auto_value is True or auto_value is None:
            workspace = Path(workspace).expanduser()
        else:
            workspace = Path(auto_value).expanduser()

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

        # Search for all *.repos files in workspace (not just depends.repos)
        for repos_file in workspace.rglob("*.repos"):
            if repos_file.is_file():
                print("ROS Jazzy: found repos file:", repos_file)
                with repos_file.open(encoding="utf-8") as f:
                    try:
                        repos_data = yaml.safe_load(f)
                    except Exception as e:
                        print(f"ROS Jazzy: failed to parse {repos_file}: {e}")
                        continue
                    if repos_data and "repositories" in repos_data and repos_data["repositories"]:
                        # Check for duplicate repository entries
                        for repo_name, repo_info in repos_data["repositories"].items():
                            if repo_name in merged_repos["repositories"]:
                                existing_info = merged_repos["repositories"][repo_name]
                                if existing_info != repo_info:
                                    print(
                                        f"ROS Jazzy: WARNING - Duplicate repo entry '{repo_name}' found in {repos_file} with conflicting details.\n"
                                        f"  Existing: {existing_info}\n"
                                        f"  New:      {repo_info}\n"
                                        f"  Keeping the first entry."
                                    )
                                # By default, keep the first entry
                                continue
                            merged_repos["repositories"][repo_name] = repo_info

        # Improved printing of merged repos
        print("ROS Jazzy: merged repos:")
        for name, info in merged_repos["repositories"].items():
            url = info.get("url", "")
            version = info.get("version", "")
            print(f"  - {name}: {url} [{version}]")

        return {
            "colcon-defaults.yaml": dat,
            "underlay_deps.sh": underlay_deps,
            "underlay_build.sh": underlay_build,
            "install_rosdeps.sh": install_rosdeps,
            "rosdep_underlay.sh": rosdep_underlay,
            "rosdep_overlay.sh": rosdep_overlay,
            "build_underlay.sh": build_underlay,
            "update_repos.sh": update_repos,
            "consolidated.repos": yaml.dump(merged_repos, default_flow_style=False),
        }

    def get_docker_args(self, cliargs) -> str:  # pylint: disable=unused-argument
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
