"""Pytest configuration and fixtures for Claude extension tests"""

# Import fixtures to make them available to all test files
from fixtures import (
    temp_claude_dir,
    valid_credentials,
    expired_credentials,
    malformed_credentials_invalid_json,
    malformed_credentials_missing_fields,
    valid_claude_config,
    valid_settings,
    temp_claude_dir_with_credentials,
    temp_claude_dir_full,
    temp_claude_dir_with_permissive_credentials,
    temp_claude_dir_with_symlink_credentials,
    temp_claude_dir_with_malformed_json,
    temp_claude_dir_with_missing_fields,
)

__all__ = [
    "temp_claude_dir",
    "valid_credentials",
    "expired_credentials",
    "malformed_credentials_invalid_json",
    "malformed_credentials_missing_fields",
    "valid_claude_config",
    "valid_settings",
    "temp_claude_dir_with_credentials",
    "temp_claude_dir_full",
    "temp_claude_dir_with_permissive_credentials",
    "temp_claude_dir_with_symlink_credentials",
    "temp_claude_dir_with_malformed_json",
    "temp_claude_dir_with_missing_fields",
]
