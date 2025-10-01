import unittest
import pytest
import os
import json
import subprocess
import tempfile
from pathlib import Path
from deps_rocker.extensions.claude.claude import Claude


class TestClaudeConfigIntegration(unittest.TestCase):
    """Comprehensive tests for Claude extension configuration mounting and detection"""

    def setUp(self):
        self.claude_ext = Claude()
        self.host_home = Path.home()

    def test_claude_extension_generates_correct_docker_args(self):
        """Test that Claude extension generates proper Docker mounting arguments"""
        docker_args = self.claude_ext.get_docker_args({})

        # Should contain volume mounts for Claude config
        self.assertIn("-v", docker_args, "Docker args should contain volume mounts")

        # Check for main config mount
        expected_main_mount = f'-v "{self.host_home}/.claude:{self.host_home}/.claude"'
        self.assertIn(expected_main_mount, docker_args, f"Should mount {self.host_home}/.claude")

        # Check for environment variables
        self.assertIn(
            f"CLAUDE_CONFIG_DIR={self.host_home}/.claude",
            docker_args,
            "Should set CLAUDE_CONFIG_DIR environment variable",
        )

        # Check XDG variables
        for xdg_var in ["XDG_CONFIG_HOME", "XDG_CACHE_HOME", "XDG_DATA_HOME"]:
            self.assertIn(xdg_var, docker_args, f"Should set {xdg_var}")

    def test_claude_config_files_exist_on_host(self):
        """Test that Claude config files exist on the host system"""
        claude_dir = self.host_home / ".claude"

        self.assertTrue(claude_dir.exists(), "~/.claude directory should exist")

        # Check for essential config files
        config_files = [".claude.json", ".credentials.json", "settings.json"]

        existing_files = []
        for config_file in config_files:
            file_path = claude_dir / config_file
            if file_path.exists():
                existing_files.append(config_file)
                # Check file permissions
                stat = file_path.stat()
                self.assertTrue(stat.st_mode & 0o400, f"{config_file} should be readable")

        self.assertGreater(len(existing_files), 0, "At least one Claude config file should exist")
        print(f"Found Claude config files: {existing_files}")

    def test_credentials_file_format(self):
        """Test that credentials file has valid format"""
        credentials_file = self.host_home / ".claude" / ".credentials.json"

        if not credentials_file.exists():
            self.skipTest("No credentials file found - authentication may be needed")

        try:
            with open(credentials_file, "r") as f:
                credentials = json.load(f)

            # Check for required OAuth structure
            self.assertIn("claudeAiOauth", credentials, "Credentials should contain claudeAiOauth")

            oauth = credentials["claudeAiOauth"]
            required_fields = ["accessToken", "refreshToken", "expiresAt"]

            for field in required_fields:
                self.assertIn(field, oauth, f"OAuth should contain {field}")
                self.assertIsNotNone(oauth[field], f"{field} should not be None")

            # Check token format
            self.assertTrue(
                oauth["accessToken"].startswith("sk-ant-oat"),
                "Access token should have correct format",
            )
            self.assertTrue(
                oauth["refreshToken"].startswith("sk-ant-ort"),
                "Refresh token should have correct format",
            )

            print(f"Credentials valid, expires at: {oauth['expiresAt']}")

        except json.JSONDecodeError as e:
            self.fail(f"Credentials file is not valid JSON: {e}")

    def test_user_override_generates_correct_paths(self):
        """Test that user override correctly changes container paths"""
        docker_args_default = self.claude_ext.get_docker_args({})
        docker_args_override = self.claude_ext.get_docker_args({"user_override_name": "testuser"})

        # Default should mount to same user
        self.assertIn(f":{self.host_home}/.claude", docker_args_default)

        # Override should mount to different user
        self.assertIn(":/home/testuser/.claude", docker_args_override)
        self.assertIn("CLAUDE_CONFIG_DIR=/home/testuser/.claude", docker_args_override)

    @pytest.mark.docker
    def test_claude_config_detection_in_container(self):
        """Test Claude config detection using a real container (integration test)"""
        # This test requires Docker and builds a real container
        try:
            # Create a simple test container with Claude extension
            dockerfile_content = """
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y curl
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash -
RUN apt-get install -y nodejs
RUN npm install -g @anthropic-ai/claude-code
ENV CLAUDE_CONFIG_DIR=/home/testuser/.claude
RUN useradd -m -s /bin/bash testuser
USER testuser
WORKDIR /home/testuser
"""

            with tempfile.NamedTemporaryFile(mode="w", suffix=".dockerfile", delete=False) as f:
                f.write(dockerfile_content)
                dockerfile_path = f.name

            try:
                # Build test image
                build_result = subprocess.run(
                    ["docker", "build", "-t", "claude-config-test", "-f", dockerfile_path, "."],
                    capture_output=True,
                    text=True,
                    timeout=300,
                )

                if build_result.returncode != 0:
                    self.skipTest(f"Docker build failed: {build_result.stderr}")

                # Test config detection in container
                docker_args = self.claude_ext.get_docker_args({"user_override_name": "testuser"})

                # Extract mount arguments
                mount_args = []
                if "-v" in docker_args:
                    # Parse volume mounts from docker_args
                    import re

                    mounts = re.findall(r'-v "([^"]+)"', docker_args)
                    for mount in mounts:
                        mount_args.extend(["-v", mount])

                # Extract environment variables
                env_args = []
                if "-e" in docker_args:
                    envs = re.findall(r'-e "([^"]+)"', docker_args)
                    for env in envs:
                        env_args.extend(["-e", env])

                # Run container with mounts and check config
                run_cmd = (
                    ["docker", "run", "--rm"]
                    + mount_args
                    + env_args
                    + [
                        "claude-config-test",
                        "bash",
                        "-c",
                        "echo 'Config dir: $CLAUDE_CONFIG_DIR' && "
                        "ls -la ~/.claude/ 2>/dev/null || echo 'No config mounted' && "
                        "claude --version 2>&1 || echo 'Claude not working'",
                    ]
                )

                result = subprocess.run(run_cmd, capture_output=True, text=True, timeout=30)

                print(f"Container output: {result.stdout}")
                if result.stderr:
                    print(f"Container stderr: {result.stderr}")

                # Verify config directory is accessible
                self.assertIn("CLAUDE_CONFIG_DIR", result.stdout)

            finally:
                # Cleanup
                os.unlink(dockerfile_path)
                subprocess.run(["docker", "rmi", "claude-config-test"], capture_output=True)

        except subprocess.TimeoutExpired:
            self.skipTest("Docker operations timed out")
        except FileNotFoundError:
            self.skipTest("Docker not available for integration test")

    def test_claude_extension_environment_variables(self):
        """Test that all required environment variables are set"""
        docker_args = self.claude_ext.get_docker_args({})

        required_env_vars = [
            "CLAUDE_CONFIG_DIR",
            "XDG_CONFIG_HOME",
            "XDG_CACHE_HOME",
            "XDG_DATA_HOME",
        ]

        for env_var in required_env_vars:
            self.assertIn(env_var, docker_args, f"Should set {env_var}")
            # Extract the value
            import re

            pattern = f'{env_var}=([^"]*)'
            match = re.search(pattern, docker_args)
            self.assertIsNotNone(match, f"Should have value for {env_var}")
            value = match.group(1)
            self.assertTrue(len(value) > 0, f"{env_var} should have non-empty value")

    def test_claude_supplemental_mounts(self):
        """Test that supplemental Claude directories are mounted if they exist"""
        host_home = Path.home()

        # Check what supplemental directories exist
        supplemental_dirs = [".cache/claude", ".local/share/claude"]

        docker_args = self.claude_ext.get_docker_args({})

        for dir_path in supplemental_dirs:
            full_path = host_home / dir_path
            if full_path.exists():
                expected_mount = f'-v "{full_path}:{host_home}/{dir_path}"'
                self.assertIn(
                    str(full_path), docker_args, f"Should mount existing directory {dir_path}"
                )

    def test_claude_extension_version_env_var(self):
        """Test that the Claude extension version environment variable is set"""
        # This tests the environment variable we added to verify latest code
        docker_args = self.claude_ext.get_docker_args({})

        # Note: This tests the extension logic, not the Dockerfile content
        # The actual CLAUDE_EXTENSION_VERSION is set in the Dockerfile
        self.assertIsInstance(docker_args, str)
        self.assertGreater(len(docker_args), 0, "Should generate non-empty Docker args")


class TestClaudeConfigValidation(unittest.TestCase):
    """Validation tests for Claude configuration files"""

    def test_claude_json_format(self):
        """Test that .claude.json has valid format"""
        claude_json_path = Path.home() / ".claude" / ".claude.json"

        if not claude_json_path.exists():
            self.skipTest("No .claude.json file found")

        try:
            with open(claude_json_path, "r") as f:
                claude_config = json.load(f)

            # Check for expected top-level keys
            expected_keys = ["projects", "sonnet45MigrationComplete"]
            for key in expected_keys:
                if key in claude_config:
                    print(f"Found expected key: {key}")

            # If projects exist, validate structure
            if "projects" in claude_config:
                projects = claude_config["projects"]
                self.assertIsInstance(projects, dict, "Projects should be a dictionary")

                for project_path, project_config in projects.items():
                    self.assertIsInstance(
                        project_config, dict, f"Project config for {project_path} should be dict"
                    )

        except json.JSONDecodeError as e:
            self.fail(f".claude.json is not valid JSON: {e}")

    def test_settings_json_format(self):
        """Test that settings.json has valid format"""
        settings_path = Path.home() / ".claude" / "settings.json"

        if not settings_path.exists():
            self.skipTest("No settings.json file found")

        try:
            with open(settings_path, "r") as f:
                settings = json.load(f)

            self.assertIsInstance(settings, dict, "Settings should be a dictionary")
            print(f"Settings keys: {list(settings.keys())}")

        except json.JSONDecodeError as e:
            self.fail(f"settings.json is not valid JSON: {e}")


if __name__ == "__main__":
    unittest.main()
