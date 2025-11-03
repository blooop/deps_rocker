import os
import hashlib
import yaml
from pathlib import Path
import logging
import em
from deps_rocker.simple_rocker_extension import SimpleRockerExtension
from deps_rocker.buildkit import is_buildkit_enabled


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

    def _create_unified_package_xml(self, dependencies):
        """Create a unified package.xml with all dependencies"""
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

    def get_files(self, cliargs) -> dict[str, str]:
        # Read the colcon-defaults template
        dat = self.get_config_file("configs/colcon-defaults.yaml").decode('utf-8')

        # Determine username for substitution
        username = self._determine_username()

        # Prepare empy substitution context
        context = {
            'name': username,
            'ros_ws_root': f'/home/{username}/overlay'
        }

        # Use empy to process the template
        try:
            # Use empy to expand the template
            processed_dat = em.expand(dat, context)

            # Print original and processed templates for debugging
            print("ROS Jazzy: Original colcon-defaults.yaml template:")
            print(dat)
            print("ROS Jazzy: Processed colcon-defaults.yaml:")
            print(processed_dat)

            # Update the template with processed content
            dat = processed_dat

        except Exception as e:
            # Log any empy processing errors
            print(f"ROS Jazzy: Failed to process colcon-defaults.yaml with empy: {e}")

        # Discover and merge repository files
        # Resolve workspace path
        workspace = self._resolve_workspace(cliargs)

        # Prepare merged repositories
        merged_repos = {"repositories": {}}

        # Search for all *.repos and *.repos.yaml files in workspace
        repos_files_found = list(workspace.rglob("*.repos")) + list(workspace.rglob("*.repos.yaml"))
        print(f"ROS Jazzy: found {len(repos_files_found)} repository files")
        for repos_file in repos_files_found:
            print("ROS Jazzy: processing:", repos_file.relative_to(workspace))

            if repos_file.is_file():
                try:
                    with repos_file.open(encoding="utf-8") as f:
                        repos_data = yaml.safe_load(f)

                    if repos_data and "repositories" in repos_data and repos_data["repositories"]:
                        # Merge repositories, handling duplicates
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
                                continue
                            merged_repos["repositories"][repo_name] = repo_info

                except Exception as e:
                    print(f"ROS Jazzy: failed to parse {repos_file}: {e}")

        # Create files dictionary
        files = {}

        # Add colcon-defaults.yaml
        files["colcon-defaults.yaml"] = dat

        # Handle consolidated.repos
        if merged_repos["repositories"]:
            print(
                f"ROS Jazzy: creating consolidated.repos with {len(merged_repos['repositories'])} repositories"
            )
            files["consolidated.repos"] = yaml.dump(merged_repos, default_flow_style=False)
        else:
            print("ROS Jazzy: no repositories found, creating empty consolidated.repos")
            files["consolidated.repos"] = "# No repositories found in workspace\nrepositories: {}\n"

        # Find and process package.xml files
        package_files = list(workspace.rglob("package.xml"))
        print(f"ROS Jazzy: found {len(package_files)} package.xml files")

        # Collect dependencies from package files
        dependencies = set()
        for pkg_file in package_files:
            try:
                import xml.etree.ElementTree as ET
                tree = ET.parse(pkg_file)
                root = tree.getroot()

                # Collect dependencies of different types
                dep_types = [
                    "depend", "build_depend", "build_export_depend",
                    "buildtool_depend", "exec_depend", "run_depend", "test_depend"
                ]
                for dep_type in dep_types:
                    dependencies.update(
                        elem.text.strip() for elem in root.findall(dep_type) if elem.text
                    )

            except Exception as e:
                print(f"ROS Jazzy: failed to parse {pkg_file}: {e}")

        # Create unified package.xml
        if dependencies:
            print(f"ROS Jazzy: creating unified package.xml with {len(dependencies)} dependencies")
            files["unified_overlay_package.xml"] = self._create_unified_package_xml(sorted(dependencies))
        else:
            print("ROS Jazzy: no dependencies found, creating empty package.xml")
            files["unified_overlay_package.xml"] = self._create_empty_package_xml()

        # Prepare script names and read script files
        script_names = [
            "underlay_deps.sh", "underlay_build.sh", "install_rosdeps.sh",
            "rosdep_underlay.sh", "rosdep_overlay.sh",
            "build_underlay.sh", "update_repos.sh"
        ]

        # Read script files
        script_dir = Path(__file__).parent
        for script_name in script_names:
            script_path = script_dir / script_name
            try:
                files[script_name] = script_path.read_text()
            except FileNotFoundError:
                # Create an empty placeholder script if not found
                files[script_name] = "#!/bin/bash\n# Placeholder script\nexit 0\n"

        return files

    def get_docker_args(self, cliargs) -> str:
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
