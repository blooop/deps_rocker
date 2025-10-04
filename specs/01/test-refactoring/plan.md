# Test Refactoring Plan

## Phase 1: Fix develop.sh Script
- Replace `grep -v` pattern with targeted `awk` command to remove specific export line
- Ensures only the intended line is removed without affecting other content

## Phase 2: Create Test Fixtures
- Create `test/fixtures.py` with reusable pytest fixtures:
  - `temp_claude_dir`: temporary ~/.claude directory with mock config files
  - `mock_credentials`: factory for creating various credential scenarios
  - `mock_config_files`: factory for creating various config file scenarios
- Use pytest fixtures to isolate tests from real filesystem

## Phase 3: Refactor test_claude_authentication.py
- Extract loop/conditional logic into helper functions
- Use pytest parametrize for tests that iterate over collections
- Add missing tests:
  - Credentials with overly permissive permissions
  - Malformed credentials (invalid JSON, missing fields)
  - Config files as symbolic links
- Move Docker integration tests to separate file (test_claude_docker_integration.py) with pytest.mark.integration

## Phase 4: Refactor test_claude_config_integration.py
- Remove duplicate logic that overlaps with test_claude_authentication.py
- Use fixtures from fixtures.py
- Add missing tests:
  - Missing config directory/files
  - User override with custom home directory
- Move Docker tests to integration suite

## Phase 5: Create Integration Test Suite
- New file: test/test_claude_docker_integration.py
- Mark with @pytest.mark.integration
- Update CI configuration to run integration tests separately (optional)

## Phase 6: Run CI and Fix Issues
- Run `pixi run ci`
- Fix any failing tests
- Ensure all tests pass before committing
