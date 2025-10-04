# Test Refactoring

## Goal
Refactor test files to follow best practices and address code review comments.

## Requirements

### Test Quality
- Remove loops and conditionals from tests (use fixtures/parametrization)
- Use temporary fixtures instead of relying on real ~/.claude directory
- Consolidate overlapping test logic between test_claude_authentication.py and test_claude_config_integration.py
- Move slow Docker integration tests to separate optional test suite

### Missing Test Coverage
- Add test for credentials with overly permissive permissions
- Add test for malformed credentials (invalid JSON, missing fields)
- Add test for config files as symbolic links
- Add test for missing config directory/files with graceful error handling
- Add test for user override with custom home directory

### Script Fixes
- Fix scripts/develop.sh to use targeted approach for removing bashrc entries (avoid grep -v pattern matching)

## Constraints
- Maintain existing test coverage
- CI must pass after changes
