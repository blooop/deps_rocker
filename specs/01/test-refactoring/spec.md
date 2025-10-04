# Test Refactoring

## Goal
Refactor test files to follow best practices and address code review comments.

## Implementation

### Test Quality ✅
- Created test/fixtures.py with reusable pytest fixtures for temp Claude configs
- Created test/test_claude_unit.py with isolated unit tests using temp fixtures
- Created test/test_claude_integration.py for slow Docker tests (marked with @pytest.mark.integration)
- Updated pyproject.toml to register integration marker
- Created conftest.py to make fixtures available to all tests
- Old test files (test_claude_authentication.py, test_claude_config_integration.py) can be deprecated

### Missing Test Coverage ✅
- Added test for credentials with overly permissive permissions
- Added test for malformed credentials (invalid JSON, missing fields)
- Added test for config files as symbolic links
- Added test for missing config directory/files
- Added test for user override with custom home directory

### Script Fixes ✅
- Fixed scripts/develop.sh to use targeted awk command instead of grep -v

## Constraints
- Maintain existing test coverage ✅
- CI must pass after changes (pending)
