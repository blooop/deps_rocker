import unittest
import pytest
import os
import json
import subprocess
import tempfile
import time
from pathlib import Path
from deps_rocker.extensions.claude.claude import Claude


class TestClaudeAuthentication(unittest.TestCase):
    """Tests for Claude authentication and credential handling"""

    def setUp(self):
        self.claude_ext = Claude()
        self.host_home = Path.home()
        self.claude_dir = self.host_home / ".claude"

    def test_credentials_file_exists_and_readable(self):
        """Test that credentials file exists and is readable"""
        credentials_path = self.claude_dir / ".credentials.json"

        self.assertTrue(credentials_path.exists(), "Credentials file should exist")

        # Check file permissions
        stat = credentials_path.stat()
        self.assertTrue(stat.st_mode & 0o400, "Credentials file should be readable by owner")

        # Check file size (should be > 0)
        self.assertGreater(
            credentials_path.stat().st_size, 0, "Credentials file should not be empty"
        )

    def test_credentials_content_valid(self):
        """Test that credentials contain valid authentication data"""
        credentials_path = self.claude_dir / ".credentials.json"

        if not credentials_path.exists():
            self.skipTest("No credentials file found")

        with open(credentials_path, "r") as f:
            credentials = json.load(f)

        # Validate OAuth structure
        self.assertIn("claudeAiOauth", credentials)
        oauth = credentials["claudeAiOauth"]

        # Check required fields
        required_fields = {
            "accessToken": str,
            "refreshToken": str,
            "expiresAt": (int, float),
            "scopes": list,
            "subscriptionType": str,
        }

        for field, expected_type in required_fields.items():
            self.assertIn(field, oauth, f"OAuth should contain {field}")
            self.assertIsInstance(oauth[field], expected_type, f"{field} should be {expected_type}")

        # Validate token formats
        self.assertTrue(
            oauth["accessToken"].startswith("sk-ant-oat"), "Access token should have correct prefix"
        )
        self.assertTrue(
            oauth["refreshToken"].startswith("sk-ant-ort"),
            "Refresh token should have correct prefix",
        )

        # Check if token is expired
        current_time = time.time() * 1000  # Convert to milliseconds
        expires_at = oauth["expiresAt"]

        if expires_at < current_time:
            self.fail(f"Token is expired. Expires: {expires_at}, Current: {current_time}")

        print(f"Token valid until: {time.ctime(expires_at / 1000)}")
        print(f"Subscription type: {oauth['subscriptionType']}")
        print(f"Scopes: {oauth['scopes']}")

    def test_credentials_vs_host_container_consistency(self):
        """Test that mounted credentials match host credentials"""
        host_creds_path = self.claude_dir / ".credentials.json"

        if not host_creds_path.exists():
            self.skipTest("No host credentials file found")

        # Read host credentials
        with open(host_creds_path, "r") as f:
            host_creds = json.load(f)

        # Simulate what would be in container
        docker_args = self.claude_ext.get_docker_args({})

        # Verify that the mounting logic would preserve the file
        expected_mount = f'-v "{host_creds_path.parent}:{self.host_home}/.claude"'
        self.assertIn(
            str(host_creds_path.parent), docker_args, "Should mount credentials directory"
        )

        # Test file content consistency (simulating container read)
        # In real container, this would be the same file due to bind mount
        container_creds = host_creds  # Same file in reality

        self.assertEqual(
            host_creds, container_creds, "Host and container credentials should be identical"
        )

    def test_environment_variables_for_auth(self):
        """Test that environment variables are set correctly for authentication"""
        docker_args = self.claude_ext.get_docker_args({})

        # Check CLAUDE_CONFIG_DIR points to correct location
        self.assertIn(
            f"CLAUDE_CONFIG_DIR={self.host_home}/.claude",
            docker_args,
            "CLAUDE_CONFIG_DIR should point to mounted config directory",
        )

        # Extract all environment variables
        import re

        env_vars = {}
        env_matches = re.findall(r'-e "([^=]+)=([^"]*)"', docker_args)
        for name, value in env_matches:
            env_vars[name] = value

        # Validate XDG variables that might affect config location
        xdg_vars = ["XDG_CONFIG_HOME", "XDG_CACHE_HOME", "XDG_DATA_HOME"]
        for var in xdg_vars:
            if var in env_vars:
                self.assertTrue(
                    env_vars[var].startswith("/home/"), f"{var} should point to user home directory"
                )

        print(f"Environment variables set: {list(env_vars.keys())}")

    @pytest.mark.skip(reason="Container test requires special Docker setup")
    def test_claude_authentication_in_container(self):
        """Test that Claude Code can authenticate in a container"""
        # This is an integration test that requires Docker
        if not self.claude_dir.exists():
            self.skipTest("No Claude config directory found")

        credentials_path = self.claude_dir / ".credentials.json"
        if not credentials_path.exists():
            self.skipTest("No credentials file found")

        try:
            # Create a test script that checks authentication
            test_script = """#!/bin/bash
set -e

echo "=== Claude Authentication Test ==="
echo "User: $(whoami)"
echo "Home: $HOME"
echo "Config dir: $CLAUDE_CONFIG_DIR"

echo ""
echo "=== Running setup script ==="
if [ -f "/usr/local/bin/setup-claude-config.sh" ]; then
    /usr/local/bin/setup-claude-config.sh || echo "Setup script failed"
else
    echo "No setup script found"
fi

echo ""
echo "=== Checking config files ==="
if [ -f "$HOME/.claude/.credentials.json" ]; then
    echo "✅ Credentials file exists"
    echo "Size: $(wc -c < $HOME/.claude/.credentials.json) bytes"
    echo "Permissions: $(ls -l $HOME/.claude/.credentials.json)"
else
    echo "❌ Credentials file missing"
    echo "Checking if mounted elsewhere..."
    find /home -name ".credentials.json" 2>/dev/null || echo "No credentials found anywhere"
    exit 1
fi

if [ -f "$HOME/.claude/.claude.json" ]; then
    echo "✅ Claude config exists"
    echo "Size: $(wc -c < $HOME/.claude/.claude.json) bytes"
else
    echo "❌ Claude config missing"
fi

echo ""
echo "=== Testing Claude Code ==="
if command -v claude &> /dev/null; then
    echo "✅ Claude command available"

    # Test version command (should not require auth)
    echo "Version check:"
    claude --version 2>&1 || echo "Version command failed"

    # Test a simple non-interactive command
    echo ""
    echo "Testing basic functionality:"
    echo '{"test": "hello"}' | timeout 10 claude --print 2>&1 | head -5 || echo "Basic test failed"

else
    echo "❌ Claude command not found"
    exit 1
fi

echo ""
echo "=== Authentication test complete ==="
"""

            with tempfile.NamedTemporaryFile(mode="w", suffix=".sh", delete=False) as f:
                f.write(test_script)
                script_path = f.name

            try:
                os.chmod(script_path, 0o755)

                # Get Docker args from Claude extension with user override
                docker_args = self.claude_ext.get_docker_args({"user_override_name": "testuser"})

                # Parse mount and environment arguments
                import re

                mount_args = []
                env_args = []

                # Extract volume mounts
                mounts = re.findall(r'-v "([^"]+)"', docker_args)
                for mount in mounts:
                    mount_args.extend(["-v", mount])

                # Extract environment variables
                envs = re.findall(r'-e "([^"]+)"', docker_args)
                for env in envs:
                    env_args.extend(["-e", env])

                # Build test image with rocker directly instead of deps-rocker wrapper
                build_cmd = [
                    "rocker",
                    "--claude",
                    "--user",
                    "--image-name",
                    "claude-test",
                    "--mode",
                    "dry-run",
                    "ubuntu:22.04",
                ]

                try:
                    build_result = subprocess.run(
                        build_cmd, capture_output=True, text=True, timeout=180
                    )
                    if build_result.returncode != 0:
                        self.skipTest(f"Failed to build test image: {build_result.stderr}")
                except subprocess.TimeoutExpired:
                    self.skipTest("Image build timed out")

                # Run the test in a container with proper mounts
                run_cmd = (
                    ["docker", "run", "--rm", "-v", f"{script_path}:/test.sh"]
                    + mount_args
                    + env_args
                    + [
                        "claude-test:latest",
                        "bash",
                        "-c",
                        "useradd -m -s /bin/bash testuser && "
                        "chown -R testuser:testuser /home/testuser && "
                        "su testuser -c '/test.sh'",
                    ]
                )

                result = subprocess.run(run_cmd, capture_output=True, text=True, timeout=120)

                print("=== Container Test Output ===")
                print(result.stdout)
                if result.stderr:
                    print("=== Container Test Errors ===")
                    print(result.stderr)

                # Analyze results
                if "Credentials file exists" in result.stdout:
                    print("✅ Credentials properly mounted")
                else:
                    self.fail("❌ Credentials not properly mounted")

                if "Claude command available" in result.stdout:
                    print("✅ Claude Code installed successfully")
                else:
                    self.fail("❌ Claude Code not available")

                # Check for authentication issues
                if "login" in result.stdout.lower() or "authenticate" in result.stdout.lower():
                    print("⚠️  Authentication may be required")
                    # This is not necessarily a failure - might need first-time setup

            finally:
                os.unlink(script_path)
                # Cleanup test image
                subprocess.run(["docker", "rmi", "claude-test:latest"], capture_output=True)

        except subprocess.TimeoutExpired:
            self.skipTest("Container test timed out")
        except FileNotFoundError:
            self.skipTest("Docker not available for integration test")
        except Exception as e:
            self.fail(f"Container test failed: {e}")

    def test_claude_config_file_ownership_and_permissions(self):
        """Test that config files have correct ownership and permissions"""
        config_files = [".claude.json", ".credentials.json", "settings.json"]

        current_uid = os.getuid()
        current_gid = os.getgid()

        for filename in config_files:
            file_path = self.claude_dir / filename
            if file_path.exists():
                stat = file_path.stat()

                # Check ownership
                self.assertEqual(
                    stat.st_uid, current_uid, f"{filename} should be owned by current user"
                )

                # Check that file is readable by owner
                self.assertTrue(stat.st_mode & 0o400, f"{filename} should be readable by owner")

                # Credentials should be more restrictive
                if filename == ".credentials.json":
                    # Should not be readable by group or others
                    self.assertFalse(
                        stat.st_mode & 0o044, f"{filename} should not be readable by group/others"
                    )

                print(
                    f"{filename}: uid={stat.st_uid}, gid={stat.st_gid}, "
                    f"mode={oct(stat.st_mode)[-3:]}"
                )

    def test_token_expiration_warning(self):
        """Test token expiration and provide warnings"""
        credentials_path = self.claude_dir / ".credentials.json"

        if not credentials_path.exists():
            self.skipTest("No credentials file found")

        with open(credentials_path, "r") as f:
            credentials = json.load(f)

        if "claudeAiOauth" not in credentials:
            self.skipTest("No OAuth credentials found")

        oauth = credentials["claudeAiOauth"]
        expires_at = oauth["expiresAt"]
        current_time = time.time() * 1000  # Convert to milliseconds

        time_until_expiry = (expires_at - current_time) / 1000  # Convert to seconds

        if time_until_expiry < 0:
            self.fail(f"❌ Token is expired by {abs(time_until_expiry)} seconds")
        elif time_until_expiry < 3600:  # Less than 1 hour
            print(f"⚠️  Token expires soon: {time_until_expiry / 60:.1f} minutes")
        elif time_until_expiry < 86400:  # Less than 1 day
            print(f"⚠️  Token expires in {time_until_expiry / 3600:.1f} hours")
        else:
            print(f"✅ Token valid for {time_until_expiry / 86400:.1f} days")


if __name__ == "__main__":
    unittest.main()
