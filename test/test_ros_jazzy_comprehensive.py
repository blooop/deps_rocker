#!/usr/bin/env python3
"""
Comprehensive ROS Jazzy Extension Test Suite

This module provides comprehensive testing for the ROS Jazzy extension, validating
all features described in the README specification.
"""

import unittest
import pytest
import io
import tempfile
import os
from pathlib import Path
from rocker.core import DockerImageGenerator, list_plugins, get_docker_client
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


@pytest.mark.docker
class TestRosJazzyComprehensive(unittest.TestCase):
    """Comprehensive tests for ROS Jazzy extension based on README specification"""

    @classmethod
    def setUpClass(cls):
        """Build a base image for testing the ROS Jazzy extension"""
        cls.base_dockerfile_tag = "testfixture_ros_jazzy_comprehensive"
        cls.base_dockerfile = """
FROM ubuntu:24.04
RUN apt-get update && apt-get install -y coreutils curl && apt-get clean
CMD [\"echo\", \"ROS Jazzy comprehensive test complete\"]
"""
        client = get_docker_client()
        iof = io.BytesIO(cls.base_dockerfile.encode())
        im = client.build(fileobj=iof, tag=cls.base_dockerfile_tag)
        for _ in im:
            pass

    @classmethod
    def tearDownClass(cls):
        """Clean up the base test image"""
        client = get_docker_client()
        try:
            client.remove_image(cls.base_dockerfile_tag, force=True)
        except Exception:
            pass

    def setUp(self):
        self.all_plugins = list_plugins()

    def _build_base_cliargs(self, **additional_args):
        """Helper to build base cliargs with common settings"""
        cliargs = {
            "base_image": self.base_dockerfile_tag,
            "extension_blacklist": [],
            "strict_extension_selection": False,
        }
        cliargs.update(additional_args)
        return cliargs

    def test_ros_jazzy_comprehensive_with_test_script(self):
        """Run the comprehensive ROS Jazzy test using the dedicated test script"""
        if "ros_jazzy" not in self.all_plugins:
            self.skipTest("ros_jazzy extension not available")

        from rocker.core import RockerExtensionManager

        # Use rocker's extension manager to properly resolve and sort dependencies
        manager = RockerExtensionManager()

        cliargs = self._build_base_cliargs(ros_jazzy=True)

        # Let rocker's extension manager handle dependency resolution and sorting
        active_extensions = manager.get_active_extensions(cliargs)

        # Add the comprehensive test script as the last extension
        test_script_path = "/home/ags/projects/deps_rocker/test/test_ros_jazzy_comprehensive.sh"
        if os.path.isfile(test_script_path):
            active_extensions.append(ScriptInjectionExtension(test_script_path))
            cliargs["command"] = "/tmp/test.sh"

        dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
        build_result = dig.build()
        self.assertEqual(build_result, 0, "ROS Jazzy comprehensive extension failed to build")

        with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
            run_result = dig.run(console_output_file=tmpfile.name)
            tmpfile.seek(0)
            output = tmpfile.read()
            print(f"DEBUG: run_result={run_result}\nContainer output:\n{output}")
            self.assertEqual(
                run_result, 0, f"ROS Jazzy comprehensive test failed. Output: {output}"
            )
        dig.clear_image()

    def test_ros_jazzy_with_dependencies_repos(self):
        """Test ROS Jazzy extension with a sample dependencies file"""
        if "ros_jazzy" not in self.all_plugins:
            self.skipTest("ros_jazzy extension not available")

        # Create a temporary workspace with a test dependencies file
        with tempfile.TemporaryDirectory() as tmpdir:
            test_workspace = Path(tmpdir)

            # Create a sample repos file with known ROS packages
            test_repos_content = """
repositories:
  unique_identifier_msgs:
    type: git
    url: https://github.com/ros2/unique_identifier_msgs.git
    version: jazzy
  geometry2:
    type: git
    url: https://github.com/ros2/geometry2.git
    version: jazzy
"""
            repos_file = test_workspace / "test.repos"
            repos_file.write_text(test_repos_content.strip())

            from rocker.core import RockerExtensionManager

            manager = RockerExtensionManager()

            # Set the auto detection to point to our test workspace
            cliargs = self._build_base_cliargs(ros_jazzy=True, auto=str(test_workspace))

            active_extensions = manager.get_active_extensions(cliargs)

            # Add the comprehensive test script
            test_script_path = "/home/ags/projects/deps_rocker/test/test_ros_jazzy_comprehensive.sh"
            if os.path.isfile(test_script_path):
                active_extensions.append(ScriptInjectionExtension(test_script_path))
                cliargs["command"] = "/tmp/test.sh"

            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()
            self.assertEqual(build_result, 0, "ROS Jazzy with dependencies failed to build")

            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                tmpfile.seek(0)
                output = tmpfile.read()
                print(f"DEBUG: run_result={run_result}\nContainer output:\n{output}")
                self.assertEqual(
                    run_result, 0, f"ROS Jazzy with dependencies test failed. Output: {output}"
                )
            dig.clear_image()

    def test_ros_jazzy_environment_variables_specification(self):
        """Test that ROS Jazzy sets all environment variables per specification"""
        if "ros_jazzy" not in self.all_plugins:
            self.skipTest("ros_jazzy extension not available")

        from rocker.core import RockerExtensionManager

        manager = RockerExtensionManager()
        cliargs = self._build_base_cliargs(ros_jazzy=True)
        active_extensions = manager.get_active_extensions(cliargs)

        # Create a test script that checks environment variables using proper bash sourcing
        env_test_script = """#!/bin/bash
set -e

echo "Testing ROS Jazzy Environment Variables Specification..."

# Use bash -c to run in a context where bashrc is properly sourced
bash -c '
    # Source the bashrc to load environment variables
    source "$HOME/.bashrc"
    
    echo "DEBUG: USERNAME=$USERNAME"
    echo "DEBUG: HOME=$HOME"
    echo "DEBUG: ROS_UNDERLAY_ROOT=$ROS_UNDERLAY_ROOT"
    echo ""
    
    # Test basic ROS variables
    [ "$ROS_DISTRO" = "jazzy" ] || { echo "ERROR: ROS_DISTRO not jazzy"; exit 1; }
    echo "✓ ROS_DISTRO=$ROS_DISTRO"
    
    # Test underlay variables per specification
    [ "$ROS_UNDERLAY_ROOT" = "$HOME/underlay" ] || { echo "ERROR: ROS_UNDERLAY_ROOT mismatch - Expected \"$HOME/underlay\", got \"$ROS_UNDERLAY_ROOT\""; exit 1; }
    echo "✓ ROS_UNDERLAY_ROOT=$ROS_UNDERLAY_ROOT"
    
    [ "$ROS_UNDERLAY_PATH" = "$HOME/underlay/src" ] || { echo "ERROR: ROS_UNDERLAY_PATH mismatch"; exit 1; }
    echo "✓ ROS_UNDERLAY_PATH=$ROS_UNDERLAY_PATH"
    
    [ "$ROS_UNDERLAY_BUILD" = "$HOME/underlay/build" ] || { echo "ERROR: ROS_UNDERLAY_BUILD mismatch"; exit 1; }
    echo "✓ ROS_UNDERLAY_BUILD=$ROS_UNDERLAY_BUILD"
    
    [ "$ROS_UNDERLAY_INSTALL" = "$HOME/underlay/install" ] || { echo "ERROR: ROS_UNDERLAY_INSTALL mismatch"; exit 1; }
    echo "✓ ROS_UNDERLAY_INSTALL=$ROS_UNDERLAY_INSTALL"
    
    # Test overlay variables per specification
    [ "$ROS_OVERLAY_ROOT" = "$HOME/overlay" ] || { echo "ERROR: ROS_OVERLAY_ROOT mismatch"; exit 1; }
    echo "✓ ROS_OVERLAY_ROOT=$ROS_OVERLAY_ROOT"
    
    [ "$ROS_WORKSPACE_ROOT" = "$HOME/overlay" ] || { echo "ERROR: ROS_WORKSPACE_ROOT mismatch"; exit 1; }
    echo "✓ ROS_WORKSPACE_ROOT=$ROS_WORKSPACE_ROOT"
    
    [ "$ROS_BUILD_BASE" = "$HOME/overlay/build" ] || { echo "ERROR: ROS_BUILD_BASE mismatch"; exit 1; }
    echo "✓ ROS_BUILD_BASE=$ROS_BUILD_BASE"
    
    [ "$ROS_INSTALL_BASE" = "$HOME/overlay/install" ] || { echo "ERROR: ROS_INSTALL_BASE mismatch"; exit 1; }
    echo "✓ ROS_INSTALL_BASE=$ROS_INSTALL_BASE"
    
    [ "$ROS_LOG_BASE" = "$HOME/overlay/log" ] || { echo "ERROR: ROS_LOG_BASE mismatch"; exit 1; }
    echo "✓ ROS_LOG_BASE=$ROS_LOG_BASE"
    
    echo "All environment variables match specification!"
'
"""

        active_extensions.append(ScriptInjectionExtension(env_test_script, is_content=True))
        cliargs["command"] = "/tmp/test.sh"
        cliargs["user"] = "ags"  # Run as ags user to get correct environment

        dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
        build_result = dig.build()
        self.assertEqual(build_result, 0, "ROS Jazzy environment test failed to build")

        with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
            run_result = dig.run(console_output_file=tmpfile.name)
            tmpfile.seek(0)
            output = tmpfile.read()
            print(f"DEBUG: run_result={run_result}\nContainer output:\n{output}")
            self.assertEqual(
                run_result, 0, f"ROS Jazzy environment variables test failed. Output: {output}"
            )

            # Verify specific output patterns
            self.assertIn("All environment variables match specification!", output)
            self.assertIn("ROS_DISTRO=jazzy", output)
            self.assertIn("ROS_UNDERLAY_ROOT=", output)
            self.assertIn("ROS_OVERLAY_ROOT=", output)

        dig.clear_image()

    def test_ros_jazzy_workspace_structure_specification(self):
        """Test that ROS Jazzy creates workspace structure per specification"""
        if "ros_jazzy" not in self.all_plugins:
            self.skipTest("ros_jazzy extension not available")

        from rocker.core import RockerExtensionManager

        manager = RockerExtensionManager()
        cliargs = self._build_base_cliargs(ros_jazzy=True)
        active_extensions = manager.get_active_extensions(cliargs)

        # Create a test script that checks workspace structure
        structure_test_script = """#!/bin/bash
set -e

echo "Testing ROS Jazzy Workspace Structure Specification..."

# Test underlay workspace structure
for dir in "$HOME/underlay" "$HOME/underlay/src" "$HOME/underlay/build" "$HOME/underlay/install" "$HOME/underlay/log"; do
    [ -d "$dir" ] || { echo "ERROR: Missing underlay directory: $dir"; exit 1; }
    echo "✓ Underlay directory exists: $dir"
done

# Test overlay workspace structure  
for dir in "$HOME/overlay" "$HOME/overlay/src" "$HOME/overlay/build" "$HOME/overlay/install" "$HOME/overlay/log"; do
    [ -d "$dir" ] || { echo "ERROR: Missing overlay directory: $dir"; exit 1; }
    echo "✓ Overlay directory exists: $dir"
done

# Test permissions
[ -r "$HOME/underlay" ] && [ -x "$HOME/underlay" ] || { echo "ERROR: Underlay not accessible"; exit 1; }
[ -r "$HOME/overlay" ] && [ -x "$HOME/overlay" ] || { echo "ERROR: Overlay not accessible"; exit 1; }

echo "All workspace structure requirements met!"
"""

        active_extensions.append(ScriptInjectionExtension(structure_test_script, is_content=True))
        cliargs["command"] = "/tmp/test.sh"

        dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
        build_result = dig.build()
        self.assertEqual(build_result, 0, "ROS Jazzy workspace structure test failed to build")

        with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
            run_result = dig.run(console_output_file=tmpfile.name)
            tmpfile.seek(0)
            output = tmpfile.read()
            print(f"DEBUG: run_result={run_result}\nContainer output:\n{output}")
            self.assertEqual(
                run_result, 0, f"ROS Jazzy workspace structure test failed. Output: {output}"
            )

            # Verify specific output patterns
            self.assertIn("All workspace structure requirements met!", output)
            self.assertIn("Underlay directory exists:", output)
            self.assertIn("Overlay directory exists:", output)

        dig.clear_image()

    def test_ros_jazzy_unified_scripts_specification(self):
        """Test that ROS Jazzy provides unified scripts per specification"""
        if "ros_jazzy" not in self.all_plugins:
            self.skipTest("ros_jazzy extension not available")

        from rocker.core import RockerExtensionManager

        manager = RockerExtensionManager()
        cliargs = self._build_base_cliargs(ros_jazzy=True)
        active_extensions = manager.get_active_extensions(cliargs)

        # Create a test script that checks unified scripts
        scripts_test_script = """#!/bin/bash
set -e

echo "Testing ROS Jazzy Unified Scripts Specification..."

# Test all unified scripts are available per specification
scripts=("rosdep_underlay.sh" "rosdep_overlay.sh" "build_underlay.sh" "update_repos.sh")

for script in "${scripts[@]}"; do
    if command -v "$script" >/dev/null 2>&1; then
        echo "✓ Script available: $script"
    else
        echo "ERROR: Script not found or not executable: $script"
        exit 1
    fi
done

echo "All unified scripts available per specification!"
"""

        active_extensions.append(ScriptInjectionExtension(scripts_test_script, is_content=True))
        cliargs["command"] = "/tmp/test.sh"

        dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
        build_result = dig.build()
        self.assertEqual(build_result, 0, "ROS Jazzy scripts test failed to build")

        with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
            run_result = dig.run(console_output_file=tmpfile.name)
            tmpfile.seek(0)
            output = tmpfile.read()
            print(f"DEBUG: run_result={run_result}\nContainer output:\n{output}")
            self.assertEqual(run_result, 0, f"ROS Jazzy scripts test failed. Output: {output}")

            # Verify specific output patterns
            self.assertIn("All unified scripts available per specification!", output)
            self.assertIn("Script available: rosdep_underlay.sh", output)
            self.assertIn("Script available: rosdep_overlay.sh", output)
            self.assertIn("Script available: build_underlay.sh", output)
            self.assertIn("Script available: update_repos.sh", output)

        dig.clear_image()


class ScriptInjectionExtension(SimpleRockerExtension):
    """Injects a test script into the Docker image and runs it as the final step."""

    name = "test_script"

    def __init__(self, script_path_or_content, is_content=False):
        if is_content:
            self.script_content = script_path_or_content
            self.script_path = None
        else:
            self.script_path = script_path_or_content
            self.script_content = None
        self.context_name = "test.sh"

    def get_snippet(self, cliargs):
        return f'COPY {self.context_name} /tmp/test.sh\nRUN chmod +x /tmp/test.sh\nCMD ["/tmp/test.sh"]'

    def get_files(self, cliargs):
        if self.script_content:
            content = self.script_content
        else:
            with open(self.script_path, "r", encoding="utf-8") as f:
                content = f.read()

        if not content.lstrip().startswith("#!/"):
            raise RuntimeError(
                "Error: test script is missing a shebang (e.g., #!/bin/bash) at the top."
            )
        return {self.context_name: content}


if __name__ == "__main__":
    unittest.main()
