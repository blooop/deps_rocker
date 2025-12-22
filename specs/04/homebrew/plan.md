# homebrew extension implementation plan

1. Create extension directory structure:
   - Create `deps_rocker/extensions/homebrew/` directory
   - Create `__init__.py`, `homebrew.py`, `homebrew_snippet.Dockerfile`, and `test.sh`

2. Implement the Homebrew extension class:
   - Inherit from `SimpleRockerExtension`
   - Set `name = "homebrew"`
   - Add dependency on `curl` extension
   - Specify required apt packages (build-essential, git, curl, procps, file)

3. Create Docker installation script:
   - Use BuildKit cache mounts for downloading Homebrew installer
   - Install Homebrew non-interactively using the official install script
   - Set appropriate environment variables (HOMEBREW_PREFIX, PATH)
   - Configure Homebrew to disable analytics and auto-update in containers

4. Add test script:
   - Verify `brew` command is available
   - Check `brew --version` works
   - Test basic functionality with `brew list`

5. Update configuration:
   - Add entry point in `pyproject.toml`
   - Add to test suite in `test/test_extensions_generic.py`

6. Run CI and verify:
   - Run `pixi run ci` to verify all tests pass
   - Fix any issues that arise
   - Commit only if CI passes
