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
        after = super().invoke_after(cliargs)
        after.update({"gemini", "claude", "codex", "user", "cwd", "workdir"})
        return after

    def _build_template_args(self, cliargs, empy_args=None) -> dict:
        """Override to add username to template context"""
        args = super()._build_template_args(cliargs, empy_args)

        username = self._determine_username()
        args["name"] = username

        return args

    def _determine_username(self) -> str:
        """Determine the username that will exist inside the container."""
        import pwd

        try:
            return pwd.getpwuid(os.getuid()).pw_name
        except (OSError, KeyError):
            return os.getenv("USER", "user")

    def _get_overlay_paths(self, username: str) -> tuple[str, str]:
        """Return overlay workspace paths for the given username."""
        container_home = f"/home/{username}"
        overlay_root = f"{container_home}/overlay"
        overlay_src = f"{overlay_root}/src"
        return overlay_root, overlay_src

    def _resolve_workspace(self, cliargs):
        """Resolve workspace path using the same logic as auto extension"""
        root = cliargs.get("auto")
        # If root is True (from --auto with no value), treat as None
        if root is True or root is None or not isinstance(root, (str, Path)):
            path = Path.cwd().expanduser().resolve()
        else:
            path = Path(root).expanduser().resolve()
        return path

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

        # Discover and merge all depends.repos files using proper workspace resolution
        workspace = self._resolve_workspace(cliargs)

        print("ROS Jazzy: searching for repository files in:", workspace)
        merged_repos = {"repositories": {}}

        # Search for all *.repos and *.repos.yaml files in workspace (not just depends.repos)
        repos_files_found = list(workspace.rglob("*.repos")) + list(workspace.rglob("*.repos.yaml"))
        print(f"ROS Jazzy: found {len(repos_files_found)} repository files")
        for repos_file in repos_files_found:
            print("ROS Jazzy: processing:", repos_file.relative_to(workspace))

        for repos_file in repos_files_found:
            if repos_file.is_file():
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

        files = {
            "colcon-defaults.yaml": dat,
            "underlay_deps.sh": underlay_deps,
            "underlay_build.sh": underlay_build,
            "install_rosdeps.sh": install_rosdeps,
            "rosdep_underlay.sh": rosdep_underlay,
            "rosdep_overlay.sh": rosdep_overlay,
            "build_underlay.sh": build_underlay,
            "update_repos.sh": update_repos,
        }

        # Always create consolidated.repos but mark it as empty if no repositories found
        if merged_repos["repositories"]:
            print(
                f"ROS Jazzy: creating consolidated.repos with {len(merged_repos['repositories'])} repositories"
            )
            files["consolidated.repos"] = yaml.dump(merged_repos, default_flow_style=False)
        else:
            print("ROS Jazzy: no repositories found, creating empty consolidated.repos")
            # Create an empty file with a comment indicating no repositories
            files["consolidated.repos"] = "# No repositories found in workspace\nrepositories: {}\n"

        # Search for package.xml files in the workspace and create unified package.xml
        unified_package_xml = self._create_unified_package_xml(workspace)
        files["unified_overlay_package.xml"] = unified_package_xml

        return files

    def _create_unified_package_xml(self, workspace):
        """Create a unified package.xml with all dependencies from workspace packages"""
        import xml.etree.ElementTree as ET

        print("ROS Jazzy: searching for package.xml files in:", workspace)
        package_files_found = list(workspace.rglob("package.xml"))
        print(f"ROS Jazzy: found {len(package_files_found)} package.xml files")

        if not package_files_found:
            print("ROS Jazzy: no package.xml files found in workspace")
            return self._create_empty_package_xml()

        # Set to collect all unique dependencies
        all_dependencies = set()

        for pkg_file in package_files_found:
            if pkg_file.is_file():
                try:
                    print(f"  - Processing: {pkg_file.relative_to(workspace)}")

                    # Parse the package.xml file
                    tree = ET.parse(pkg_file)
                    root = tree.getroot()

                    # Extract all dependency types
                    for dep_type in [
                        "depend",
                        "build_depend",
                        "build_export_depend",
                        "buildtool_depend",
                        "buildtool_export_depend",
                        "exec_depend",
                        "run_depend",
                        "test_depend",
                    ]:
                        for dep_elem in root.findall(dep_type):
                            dep_name = dep_elem.text.strip() if dep_elem.text else None
                            if dep_name:
                                all_dependencies.add(dep_name)

                except Exception as e:
                    print(f"ROS Jazzy: failed to parse {pkg_file}: {e}")
                    continue

        if not all_dependencies:
            print("ROS Jazzy: no dependencies found in package files")
            return self._create_empty_package_xml()

        # Sort dependencies alphabetically for consistent caching
        sorted_dependencies = sorted(all_dependencies)
        print(f"ROS Jazzy: consolidated {len(sorted_dependencies)} unique dependencies")
        for dep in sorted_dependencies[:10]:  # Show first 10
            print(f"  - {dep}")
        if len(sorted_dependencies) > 10:
            print(f"  ... and {len(sorted_dependencies) - 10} more")

        # Create unified package.xml
        return self._generate_unified_package_xml(sorted_dependencies)

    def _create_empty_package_xml(self):
        """Create an empty package.xml when no dependencies are found"""
        return """<?xml version="1.0"?>
<package format="3">
  <name>empty_overlay_deps</name>
  <version>1.0.0</version>
  <description>Empty package for overlay dependencies - no packages found</description>
  <maintainer email="rocker@example.com">rocker</maintainer>
  <license>MIT</license>
</package>
"""

    def _generate_unified_package_xml(self, dependencies):
        """Generate a unified package.xml with all dependencies"""
        xml_content = """<?xml version="1.0"?>
<package format="3">
  <name>unified_overlay_deps</name>
  <version>1.0.0</version>
  <description>Unified package containing all dependencies from workspace packages</description>
  <maintainer email="rocker@example.com">rocker</maintainer>
  <license>MIT</license>

"""

        # Add all dependencies as <depend> tags
        depend_tags = [f"  <depend>{dep}</depend>" for dep in dependencies]
        xml_content += "\n".join(depend_tags) + "\n"

        xml_content += "</package>\n"

        return xml_content

    def get_docker_args(self, cliargs) -> str:  # pylint: disable=unused-argument
        """Configure runtime environment, workspace mount, and working directory."""
        username = self._determine_username()
        overlay_root, overlay_src = self._get_overlay_paths(username)
        workspace = self._resolve_workspace(cliargs)

        docker_args: list[str] = []

        def _has_volume_target(target: str) -> bool:
            volumes = cliargs.get("volume") or []
            for entry in volumes:
                if not isinstance(entry, (list, tuple)):
                    continue
                for spec in entry:
                    if not isinstance(spec, str):
                        continue
                    parts = spec.split(":")
                    if len(parts) >= 2 and parts[1] == target:
                        return True
            return False

        if workspace and workspace.exists() and not _has_volume_target(overlay_src):
            docker_args.append(f'-v "{str(workspace)}:{overlay_src}"')

        docker_args.append(f'-w "{overlay_root}"')

        ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID")
        if ROS_DOMAIN_ID is None:
            host_username = os.environ.get("USER")
            if host_username:
                # Hash the username
                hashed_value = int(hashlib.sha256(host_username.encode()).hexdigest(), 16)
                # Scale the hash to a value between 2 and 99
                ROS_DOMAIN_ID = str((hashed_value % 98) + 2)
            else:
                raise ValueError("Unable to determine username and no ROS_DOMAIN_ID provided.")

        docker_args.append(f"--env ROS_DOMAIN_ID={ROS_DOMAIN_ID}")

        return " " + " ".join(docker_args)
