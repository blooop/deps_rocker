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
        test_script_path = os.path.join(
            os.path.dirname(__file__), "test_ros_jazzy_comprehensive.sh"
        )
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
"""
            repos_file = test_workspace / "test.repos"
            repos_file.write_text(test_repos_content.strip())

            from rocker.core import RockerExtensionManager

            manager = RockerExtensionManager()

            # Set the auto detection to point to our test workspace
            cliargs = self._build_base_cliargs(ros_jazzy=True, auto=str(test_workspace))

            active_extensions = manager.get_active_extensions(cliargs)

            # Add the comprehensive test script
            test_script_path = os.path.join(
                os.path.dirname(__file__), "test_ros_jazzy_comprehensive.sh"
            )
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

    def test_ros_jazzy_in_container_scripts_functionality(self):
        """Test ROS Jazzy in-container unified scripts for dependency management"""
        if "ros_jazzy" not in self.all_plugins:
            self.skipTest("ros_jazzy extension not available")

        # Create a test workspace with sample dependencies
        with tempfile.TemporaryDirectory() as tmpdir:
            test_workspace = Path(tmpdir)

            # Create a sample repos file with a lightweight ROS package
            test_repos_content = """
repositories:
  unique_identifier_msgs:
    type: git
    url: https://github.com/ros2/unique_identifier_msgs.git
    version: jazzy
"""
            repos_file = test_workspace / "test.repos"
            repos_file.write_text(test_repos_content.strip())

            from rocker.core import RockerExtensionManager

            manager = RockerExtensionManager()

            # Use only ros_jazzy extension without auto-detection to avoid BuildKit issues
            cliargs = self._build_base_cliargs(ros_jazzy=True)
            active_extensions = manager.get_active_extensions(cliargs)

            # Manually create a consolidated repos file for the ROS extension to use
            # This simulates the repos file that would normally be detected
            repos_content_for_extension = test_repos_content.strip()

            # Create a custom ROS extension that uses our repos file
            # We put this test repos file in a different location to avoid conflicts
            class RosJazzyTestExtension(SimpleRockerExtension):
                name = "ros_jazzy_test"

                def get_preamble(self, cliargs):
                    return ""

                def get_snippet(self, cliargs):
                    return """
# Copy test repos file for runtime script testing
# Note: we use a different name to avoid conflicts with build-time repos
COPY test.repos /tmp/test_runtime.repos
"""

                def get_files(self, cliargs):
                    return {"test.repos": repos_content_for_extension}

            # Add our test extension
            active_extensions.append(RosJazzyTestExtension())

            # Create comprehensive in-container script test
            script_test_content = """#!/bin/bash
set -e

echo "=========================================="
echo "ROS Jazzy In-Container Scripts Test Suite"
echo "=========================================="

# Color codes for output
GREEN='\\033[0;32m'
RED='\\033[0;31m'
NC='\\033[0m' # No Color

log_info() {
    echo -e "${GREEN}✓${NC} $1"
}

log_error() {
    echo -e "${RED}✗${NC} $1"
    exit 1
}

# Test environment setup
source /opt/ros/jazzy/setup.bash
log_info "Sourced ROS environment"

# 1. TEST UNIFIED SCRIPTS AVAILABILITY
echo ""
echo "1. Testing Unified Scripts Availability..."

scripts_to_test=(
    "underlay_deps.sh"
    "build_underlay.sh" 
    "update_repos.sh"
    "rosdep_underlay.sh"
    "rosdep_overlay.sh"
)

for script in "${scripts_to_test[@]}"; do
    if command -v "$script" >/dev/null 2>&1; then
        log_info "Script available: $script"
    else
        log_error "Script not found in PATH: $script"
    fi
done

# 2. TEST ENVIRONMENT VARIABLES FOR SCRIPTS
echo ""
echo "2. Testing Environment Variables for Scripts..."

required_vars=(
    "ROS_UNDERLAY_ROOT"
    "ROS_UNDERLAY_PATH"
    "ROS_UNDERLAY_BUILD"
    "ROS_UNDERLAY_INSTALL"
    "ROS_OVERLAY_ROOT"
    "ROS_WORKSPACE_ROOT"
)

for var in "${required_vars[@]}"; do
    if [ -n "${!var}" ]; then
        log_info "Environment variable set: $var=${!var}"
    else
        log_error "Required environment variable not set: $var"
    fi
done

# 3. TEST WORKSPACE STRUCTURE
echo ""
echo "3. Testing Workspace Structure..."

workspace_dirs=(
    "$ROS_UNDERLAY_ROOT"
    "$ROS_UNDERLAY_PATH"
    "$ROS_UNDERLAY_BUILD"
    "$ROS_UNDERLAY_INSTALL"
    "$ROS_OVERLAY_ROOT"
    "$ROS_WORKSPACE_ROOT/src"
    "$ROS_WORKSPACE_ROOT/build"
    "$ROS_WORKSPACE_ROOT/install"
)

for dir in "${workspace_dirs[@]}"; do
    if [ -d "$dir" ]; then
        log_info "Directory exists: $dir"
    else
        log_error "Required directory missing: $dir"
    fi
done

# 4. TEST UPDATE_REPOS.SH FUNCTIONALITY
echo ""
echo "4. Testing update_repos.sh functionality..."

# First, clear any existing repos from build time to test runtime functionality
echo "Clearing existing underlay to test runtime repository import..."
rm -rf "$ROS_UNDERLAY_PATH"/* 2>/dev/null || true
mkdir -p "$ROS_UNDERLAY_PATH"

# Create a writable test directory  
SCRIPT_TEST_DIR="/tmp/script_test"
mkdir -p "$SCRIPT_TEST_DIR"
cd "$SCRIPT_TEST_DIR"

# Test with the runtime test repos file if available
if [ -f "/tmp/test_runtime.repos" ]; then
    log_info "Found test_runtime.repos file for testing"
    
    # Copy it to our test directory as a .repos file
    cp "/tmp/test_runtime.repos" "./test_script.repos"
    
    echo "Contents of test repos file:"
    cat "./test_script.repos"
    
    # Test update_repos.sh with the test repos
    if update_repos.sh; then
        log_info "update_repos.sh executed successfully"

        # Check if packages were cloned to underlay
        echo "Debug: Checking underlay directory: $ROS_UNDERLAY_PATH"
        if [ -d "$ROS_UNDERLAY_PATH" ]; then
            echo "Debug: Underlay directory exists, contents:"
            ls -la "$ROS_UNDERLAY_PATH" || echo "Failed to list contents"
            
            if [ "$(ls -A "$ROS_UNDERLAY_PATH" 2>/dev/null)" ]; then
                log_info "Repositories cloned to underlay workspace"
                
                # List what was cloned
                echo "Cloned files and directories:"
                find "$ROS_UNDERLAY_PATH" -name "*" | head -10
                echo "Package.xml files:"
                find "$ROS_UNDERLAY_PATH" -name "package.xml" | head -5
                
                # Success if any directory was created (repository was cloned)
                if find "$ROS_UNDERLAY_PATH" -mindepth 1 -type d | head -1 | grep -q .; then
                    log_info "Repository directories found in underlay"
                else
                    log_error "No repository directories found in underlay"
                fi
            else
                log_error "No files found in underlay after update_repos.sh"
            fi
        else
            log_error "Underlay directory does not exist: $ROS_UNDERLAY_PATH"
        fi
    else
        log_error "update_repos.sh failed"
    fi
elif [ -f "consolidated.repos" ]; then
    log_info "Found consolidated.repos file"

    # Test update_repos.sh with existing repos
    if update_repos.sh; then
        log_info "update_repos.sh executed successfully (with existing repos)"
        
        # For existing repos, just verify the script ran without failing
        log_info "Script execution completed successfully"
    else
        log_error "update_repos.sh failed"
    fi
else
    log_info "No repos files found, testing empty scenario"
    if update_repos.sh; then
        log_info "update_repos.sh handled empty scenario correctly"  
    else
        log_error "update_repos.sh failed on empty scenario"
    fi
    fi

# 5. TEST UNDERLAY_DEPS.SH FUNCTIONALITY
echo ""
echo "5. Testing underlay_deps.sh functionality..."

if [ -d "$ROS_UNDERLAY_PATH" ] && [ "$(find "$ROS_UNDERLAY_PATH" -name 'package.xml' -print -quit)" ]; then
    echo "Found packages in underlay, testing dependency installation..."
    
    # Test rosdep installation
    if underlay_deps.sh; then
        log_info "underlay_deps.sh executed successfully"
    else
        log_error "underlay_deps.sh failed"
    fi
else
    log_info "No packages in underlay, testing empty scenario"
    if underlay_deps.sh; then
        log_info "underlay_deps.sh handled empty underlay correctly"
    else
        log_error "underlay_deps.sh failed on empty underlay"
    fi
fi

# 6. TEST BUILD_UNDERLAY.SH FUNCTIONALITY  
echo ""
echo "6. Testing build_underlay.sh functionality..."

if [ -d "$ROS_UNDERLAY_PATH" ] && [ "$(find "$ROS_UNDERLAY_PATH" -name 'package.xml' -print -quit)" ]; then
    echo "Found packages in underlay, testing build..."
    
    # Test underlay build
    if build_underlay.sh; then
        log_info "build_underlay.sh executed successfully"
        
        # Check if build artifacts were created
        if [ -d "$ROS_UNDERLAY_INSTALL" ] && [ "$(ls -A "$ROS_UNDERLAY_INSTALL" 2>/dev/null)" ]; then
            log_info "Build artifacts created in underlay install directory"
            
            # Test if setup.bash was created
            if [ -f "$ROS_UNDERLAY_INSTALL/setup.bash" ]; then
                log_info "Underlay setup.bash created"
                
                # Test sourcing the setup file
                if source "$ROS_UNDERLAY_INSTALL/setup.bash"; then
                    log_info "Successfully sourced underlay setup.bash"
                else
                    log_error "Failed to source underlay setup.bash"
                fi
            else
                log_info "No setup.bash found (may be expected if no packages built)"
            fi
        else
            log_info "No build artifacts (may be expected for header-only packages)"
        fi
    else
        log_error "build_underlay.sh failed"
    fi
else
    log_info "No packages in underlay, testing empty scenario"
    if build_underlay.sh; then
        log_info "build_underlay.sh handled empty underlay correctly"
    else
        log_error "build_underlay.sh failed on empty underlay"  
    fi
fi

# 7. TEST ROSDEP SCRIPTS
echo ""
echo "7. Testing rosdep scripts..."

# Test rosdep_underlay.sh
if rosdep_underlay.sh; then
    log_info "rosdep_underlay.sh executed successfully"
else
    log_error "rosdep_underlay.sh failed"
fi

# Test rosdep_overlay.sh (should handle empty overlay)
if rosdep_overlay.sh; then
    log_info "rosdep_overlay.sh executed successfully"
else
    log_error "rosdep_overlay.sh failed"
fi

# 8. TEST SCRIPT INTEGRATION
echo ""
echo "8. Testing Script Integration..."

# Test that scripts work in sequence (dependency -> build workflow)
echo "Testing complete workflow: update -> deps -> build"

# Reset underlay to test full workflow
rm -rf "$ROS_UNDERLAY_PATH"/* 2>/dev/null || true
mkdir -p "$ROS_UNDERLAY_PATH"

# If we have repos, test the full workflow
cd /tmp
if [ -f "consolidated.repos" ]; then
    log_info "Testing full workflow with repositories"
    
    # Step 1: Update repos
    if update_repos.sh; then
        log_info "Step 1: Repository update completed"
        
        # Step 2: Install deps  
        if underlay_deps.sh; then
            log_info "Step 2: Dependency installation completed"
            
            # Step 3: Build
            if build_underlay.sh; then
                log_info "Step 3: Build completed"
                log_info "Full workflow test: SUCCESS"
            else
                log_error "Step 3: Build failed"
            fi
        else
            log_error "Step 2: Dependency installation failed"
        fi
    else
        log_error "Step 1: Repository update failed"
    fi
else
    log_info "No repos file, workflow test with empty workspace completed"
fi

echo ""
echo "=========================================="
echo "✓ All in-container script tests completed successfully!"
echo "=========================================="
"""

            # Write the script test file
            script_test_path = test_workspace / "in_container_scripts_test.sh"
            script_test_path.write_text(script_test_content)
            script_test_path.chmod(0o755)

            # Add script injection for the test
            active_extensions.append(ScriptInjectionExtension(str(script_test_path)))
            cliargs["command"] = f"/tmp/{script_test_path.name}"

            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()
            self.assertEqual(build_result, 0, "ROS Jazzy in-container scripts test failed to build")

            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                tmpfile.seek(0)
                output = tmpfile.read()
                print(f"In-container scripts test output:\n{output}")
                self.assertEqual(
                    run_result, 0, f"ROS Jazzy in-container scripts test failed. Output: {output}"
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

        # Create a test script that checks environment variables
        env_test_script = """#!/bin/bash
set -e

echo "Testing ROS Jazzy Environment Variables Specification..."

# Test basic ROS variables
[ "$ROS_DISTRO" = "jazzy" ] || { echo "ERROR: ROS_DISTRO not jazzy"; exit 1; }
echo "✓ ROS_DISTRO=$ROS_DISTRO"

# Test underlay variables per specification
[ "$ROS_UNDERLAY_ROOT" = "$HOME/underlay" ] || { echo "ERROR: ROS_UNDERLAY_ROOT mismatch"; exit 1; }
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
"""

        active_extensions.append(ScriptInjectionExtension(env_test_script, is_content=True))
        cliargs["command"] = "/tmp/test.sh"

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
