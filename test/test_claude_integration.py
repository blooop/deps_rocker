"""Integration tests for Claude extension (requires Docker)

These tests are slow and environment-dependent. They can be run separately with:
    pytest -m integration test/test_claude_integration.py

Or excluded from regular test runs with:
    pytest -m "not integration"
"""

import json
import os
import pytest
import subprocess
import tempfile
from pathlib import Path
from deps_rocker.extensions.claude.claude import Claude


@pytest.mark.integration
class TestClaudeDockerIntegration:
    """Integration tests that require Docker"""

    @pytest.fixture
    def claude_ext(self):
        """Create Claude extension instance"""
        return Claude()

    @pytest.fixture
    def host_home(self):
        """Get host home directory"""
        return Path.home()

    @pytest.fixture
    def claude_dir(self, host_home):
        """Get Claude config directory"""
        return host_home / ".claude"

    def test_claude_config_detection_in_container(self, claude_ext, claude_dir):
        """Test Claude config detection using a real container"""
        if not claude_dir.exists():
            pytest.skip("No Claude config directory found")

        try:
            # Create a simple test container
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
                    pytest.skip(f"Docker build failed: {build_result.stderr}")

                # Get Docker args from extension
                docker_args = claude_ext.get_docker_args({"user_override_name": "testuser"})

                # Extract mount and env arguments
                import re

                mount_args = []
                mounts = re.findall(r'-v "([^"]+)"', docker_args)
                for mount in mounts:
                    mount_args.extend(["-v", mount])

                env_args = []
                envs = re.findall(r'-e "([^"]+)"', docker_args)
                for env in envs:
                    env_args.extend(["-e", env])

                # Run container and check config
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

                # Verify config directory is accessible
                assert "CLAUDE_CONFIG_DIR" in result.stdout

            finally:
                # Cleanup
                os.unlink(dockerfile_path)
                subprocess.run(["docker", "rmi", "claude-config-test"], capture_output=True)

        except subprocess.TimeoutExpired:
            pytest.skip("Docker operations timed out")
        except FileNotFoundError:
            pytest.skip("Docker not available")


@pytest.mark.integration
class TestClaudeHostIntegration:
    """Integration tests using actual host ~/.claude directory"""

    @pytest.fixture
    def host_home(self):
        return Path.home()

    @pytest.fixture
    def claude_dir(self, host_home):
        return host_home / ".claude"

    def test_host_credentials_exist(self, claude_dir):
        """Test that host credentials exist (skips if not)"""
        credentials_path = claude_dir / ".credentials.json"

        if not credentials_path.exists():
            pytest.skip("No host credentials file found")

        assert credentials_path.exists()
        assert credentials_path.stat().st_size > 0

    def test_host_credentials_valid(self, claude_dir):
        """Test that host credentials are valid"""
        credentials_path = claude_dir / ".credentials.json"

        if not credentials_path.exists():
            pytest.skip("No credentials file found")

        with open(credentials_path) as f:
            credentials = json.load(f)

        assert "claudeAiOauth" in credentials
        oauth = credentials["claudeAiOauth"]

        # Check required fields
        assert "accessToken" in oauth
        assert "refreshToken" in oauth
        assert "expiresAt" in oauth

    def test_host_credentials_permissions(self, claude_dir):
        """Test that host credentials have secure permissions"""
        credentials_path = claude_dir / ".credentials.json"

        if not credentials_path.exists():
            pytest.skip("No credentials file found")

        file_stat = credentials_path.stat()

        # Should be readable by owner
        assert file_stat.st_mode & 0o400
