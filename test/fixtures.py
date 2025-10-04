"""Pytest fixtures for Claude extension tests"""

import json
import pytest
import time


@pytest.fixture
def temp_claude_dir(tmp_path):
    """Create a temporary .claude directory with basic structure"""
    claude_dir = tmp_path / ".claude"
    claude_dir.mkdir()
    return claude_dir


@pytest.fixture
def valid_credentials():
    """Return valid Claude OAuth credentials structure"""
    return {
        "claudeAiOauth": {
            "accessToken": "sk-ant-oat-test-token-12345",
            "refreshToken": "sk-ant-ort-test-refresh-67890",
            "expiresAt": int((time.time() + 86400) * 1000),  # Expires in 24 hours
            "scopes": ["chat", "api"],
            "subscriptionType": "pro",
        }
    }


@pytest.fixture
def expired_credentials():
    """Return expired Claude OAuth credentials"""
    return {
        "claudeAiOauth": {
            "accessToken": "sk-ant-oat-expired-token",
            "refreshToken": "sk-ant-ort-expired-refresh",
            "expiresAt": int((time.time() - 3600) * 1000),  # Expired 1 hour ago
            "scopes": ["chat"],
            "subscriptionType": "free",
        }
    }


@pytest.fixture
def malformed_credentials_invalid_json():
    """Return malformed JSON for credentials"""
    return '{"claudeAiOauth": {"accessToken": "sk-ant-oat-test"'  # Missing closing braces


@pytest.fixture
def malformed_credentials_missing_fields():
    """Return credentials with missing required fields"""
    return {
        "claudeAiOauth": {
            "accessToken": "sk-ant-oat-test-token",
            # Missing refreshToken, expiresAt, scopes, subscriptionType
        }
    }


@pytest.fixture
def valid_claude_config():
    """Return valid .claude.json structure"""
    return {
        "projects": {},
        "sonnet45MigrationComplete": True,
    }


@pytest.fixture
def valid_settings():
    """Return valid settings.json structure"""
    return {
        "theme": "dark",
        "notifications": True,
    }


@pytest.fixture
def temp_claude_dir_with_credentials(temp_claude_dir, valid_credentials):
    """Create temp Claude dir with valid credentials file"""
    credentials_path = temp_claude_dir / ".credentials.json"
    credentials_path.write_text(json.dumps(valid_credentials))
    credentials_path.chmod(0o600)
    return temp_claude_dir


@pytest.fixture
def temp_claude_dir_full(temp_claude_dir, valid_credentials, valid_claude_config, valid_settings):
    """Create temp Claude dir with all config files"""
    # Credentials
    credentials_path = temp_claude_dir / ".credentials.json"
    credentials_path.write_text(json.dumps(valid_credentials))
    credentials_path.chmod(0o600)

    # Claude config
    config_path = temp_claude_dir / ".claude.json"
    config_path.write_text(json.dumps(valid_claude_config))

    # Settings
    settings_path = temp_claude_dir / "settings.json"
    settings_path.write_text(json.dumps(valid_settings))

    return temp_claude_dir


@pytest.fixture
def temp_claude_dir_with_permissive_credentials(temp_claude_dir, valid_credentials):
    """Create temp Claude dir with world-readable credentials (insecure)"""
    credentials_path = temp_claude_dir / ".credentials.json"
    credentials_path.write_text(json.dumps(valid_credentials))
    credentials_path.chmod(0o644)  # World-readable
    return temp_claude_dir


@pytest.fixture
def temp_claude_dir_with_symlink_credentials(tmp_path, valid_credentials):
    """Create temp Claude dir where credentials is a symlink"""
    # Create actual credentials file in a different location
    actual_creds = tmp_path / "actual_credentials.json"
    actual_creds.write_text(json.dumps(valid_credentials))
    actual_creds.chmod(0o600)

    # Create Claude dir with symlink
    claude_dir = tmp_path / ".claude"
    claude_dir.mkdir()

    credentials_link = claude_dir / ".credentials.json"
    credentials_link.symlink_to(actual_creds)

    return claude_dir


@pytest.fixture
def temp_claude_dir_with_malformed_json(temp_claude_dir, malformed_credentials_invalid_json):
    """Create temp Claude dir with malformed JSON credentials"""
    credentials_path = temp_claude_dir / ".credentials.json"
    credentials_path.write_text(malformed_credentials_invalid_json)
    return temp_claude_dir


@pytest.fixture
def temp_claude_dir_with_missing_fields(temp_claude_dir, malformed_credentials_missing_fields):
    """Create temp Claude dir with credentials missing required fields"""
    credentials_path = temp_claude_dir / ".credentials.json"
    credentials_path.write_text(json.dumps(malformed_credentials_missing_fields))
    return temp_claude_dir
