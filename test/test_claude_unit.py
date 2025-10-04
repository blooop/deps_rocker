"""Unit tests for Claude extension using temporary fixtures"""

import json
import pytest
import stat
import time
from deps_rocker.extensions.claude.claude import Claude


class TestClaudeCredentials:
    """Tests for Claude credentials handling"""

    def test_valid_credentials_structure(self, temp_claude_dir_with_credentials):
        """Test that valid credentials have correct structure"""
        credentials_path = temp_claude_dir_with_credentials / ".credentials.json"

        assert credentials_path.exists()

        with open(credentials_path) as f:
            credentials = json.load(f)

        assert "claudeAiOauth" in credentials
        oauth = credentials["claudeAiOauth"]

        # Check required fields
        assert "accessToken" in oauth
        assert "refreshToken" in oauth
        assert "expiresAt" in oauth
        assert "scopes" in oauth
        assert "subscriptionType" in oauth

        # Check types
        assert isinstance(oauth["accessToken"], str)
        assert isinstance(oauth["refreshToken"], str)
        assert isinstance(oauth["expiresAt"], (int, float))
        assert isinstance(oauth["scopes"], list)
        assert isinstance(oauth["subscriptionType"], str)

    def test_credentials_file_permissions(self, temp_claude_dir_with_credentials):
        """Test that credentials file has correct permissions"""
        credentials_path = temp_claude_dir_with_credentials / ".credentials.json"
        file_stat = credentials_path.stat()

        # Should be readable by owner
        assert file_stat.st_mode & stat.S_IRUSR

        # Should NOT be readable by group or others (secure)
        assert not (file_stat.st_mode & stat.S_IRGRP)
        assert not (file_stat.st_mode & stat.S_IROTH)

    def test_credentials_with_overly_permissive_permissions(
        self, temp_claude_dir_with_permissive_credentials
    ):
        """Test detection of credentials with overly permissive permissions"""
        credentials_path = temp_claude_dir_with_permissive_credentials / ".credentials.json"
        file_stat = credentials_path.stat()

        # File should be world-readable for this test
        assert file_stat.st_mode & stat.S_IROTH

        # This is a security issue - credentials should not be world-readable
        # A proper implementation should warn or fail
        is_too_permissive = bool(file_stat.st_mode & (stat.S_IRGRP | stat.S_IROTH))
        assert is_too_permissive, "Test setup: credentials should be too permissive"

    def test_credentials_malformed_json(self, temp_claude_dir_with_malformed_json):
        """Test handling of malformed JSON credentials"""
        credentials_path = temp_claude_dir_with_malformed_json / ".credentials.json"

        with pytest.raises(json.JSONDecodeError):
            with open(credentials_path) as f:
                json.load(f)

    def test_credentials_missing_required_fields(self, temp_claude_dir_with_missing_fields):
        """Test handling of credentials with missing required fields"""
        credentials_path = temp_claude_dir_with_missing_fields / ".credentials.json"

        with open(credentials_path) as f:
            credentials = json.load(f)

        oauth = credentials.get("claudeAiOauth", {})

        # Check that required fields are missing
        assert "refreshToken" not in oauth
        assert "expiresAt" not in oauth

    def test_token_format_validation(self, temp_claude_dir_with_credentials):
        """Test that tokens have correct format"""
        credentials_path = temp_claude_dir_with_credentials / ".credentials.json"

        with open(credentials_path) as f:
            credentials = json.load(f)

        oauth = credentials["claudeAiOauth"]

        # Validate token prefixes
        assert oauth["accessToken"].startswith("sk-ant-oat")
        assert oauth["refreshToken"].startswith("sk-ant-ort")

    def test_token_expiration_check(self, temp_claude_dir_with_credentials):
        """Test token expiration checking"""
        credentials_path = temp_claude_dir_with_credentials / ".credentials.json"

        with open(credentials_path) as f:
            credentials = json.load(f)

        oauth = credentials["claudeAiOauth"]
        current_time = time.time() * 1000

        # Token should not be expired (fixture creates future expiry)
        assert oauth["expiresAt"] > current_time

    def test_expired_token_detection(self, temp_claude_dir, expired_credentials):
        """Test detection of expired tokens"""
        credentials_path = temp_claude_dir / ".credentials.json"
        credentials_path.write_text(json.dumps(expired_credentials))

        with open(credentials_path) as f:
            credentials = json.load(f)

        oauth = credentials["claudeAiOauth"]
        current_time = time.time() * 1000

        # Token should be expired
        assert oauth["expiresAt"] < current_time

    def test_credentials_as_symlink(self, temp_claude_dir_with_symlink_credentials):
        """Test that credentials work when symlinked"""
        credentials_path = temp_claude_dir_with_symlink_credentials / ".credentials.json"

        # Should be a symlink
        assert credentials_path.is_symlink()

        # Should still be readable
        assert credentials_path.exists()

        # Can read credentials through symlink
        with open(credentials_path) as f:
            credentials = json.load(f)

        assert "claudeAiOauth" in credentials

        # Check permissions of the target file (not the link itself)
        target_stat = credentials_path.stat()  # Follows symlink
        assert target_stat.st_mode & stat.S_IRUSR


class TestClaudeConfigFiles:
    """Tests for Claude config files"""

    def test_claude_json_structure(self, temp_claude_dir_full):
        """Test .claude.json file structure"""
        config_path = temp_claude_dir_full / ".claude.json"

        assert config_path.exists()

        with open(config_path) as f:
            config = json.load(f)

        assert isinstance(config, dict)
        assert "projects" in config or "sonnet45MigrationComplete" in config

    def test_settings_json_structure(self, temp_claude_dir_full):
        """Test settings.json file structure"""
        settings_path = temp_claude_dir_full / "settings.json"

        assert settings_path.exists()

        with open(settings_path) as f:
            settings = json.load(f)

        assert isinstance(settings, dict)

    def test_missing_config_directory(self, tmp_path):
        """Test handling of missing config directory"""
        non_existent_dir = tmp_path / "non_existent" / ".claude"

        assert not non_existent_dir.exists()

    def test_missing_config_files(self, temp_claude_dir):
        """Test handling of missing config files"""
        credentials_path = temp_claude_dir / ".credentials.json"
        config_path = temp_claude_dir / ".claude.json"

        assert not credentials_path.exists()
        assert not config_path.exists()


class TestClaudeExtensionDockerArgs:
    """Tests for Claude extension Docker arguments generation"""

    def test_docker_args_default_user(self):
        """Test Docker args with default user"""
        claude_ext = Claude()
        docker_args = claude_ext.get_docker_args({})

        assert isinstance(docker_args, str)
        assert len(docker_args) > 0
        assert "-v" in docker_args
        assert "CLAUDE_CONFIG_DIR" in docker_args

    def test_docker_args_user_override(self):
        """Test Docker args with user override"""
        claude_ext = Claude()
        docker_args = claude_ext.get_docker_args({"user_override_name": "testuser"})

        assert ":/home/testuser/.claude" in docker_args
        assert "CLAUDE_CONFIG_DIR=/home/testuser/.claude" in docker_args

    def test_docker_args_user_override_custom_home(self):
        """Test Docker args with custom home directory"""
        claude_ext = Claude()

        # Assuming the extension accepts a custom home parameter
        # If not, this test documents the desired behavior
        custom_home = "/custom/home/testuser"
        docker_args = claude_ext.get_docker_args({"user_override_name": "testuser"})

        # This tests current behavior; may need implementation update
        # to support custom home directories
        assert "testuser" in docker_args

    def test_docker_args_environment_variables(self):
        """Test that required environment variables are set"""
        claude_ext = Claude()
        docker_args = claude_ext.get_docker_args({})

        # Check that CLAUDE_CONFIG_DIR is set
        assert "CLAUDE_CONFIG_DIR" in docker_args
