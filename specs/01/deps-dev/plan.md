# Plan for deps-devtools Extension

1. Create extension directory: deps_rocker/extensions/deps_devtools/
2. Implement deps_devtools.py:
   - Inherit from SimpleRockerExtension
   - Set name to "deps-devtools"
   - depends_on_extension = ("fzf",)
   - Docstring: Installs ripgrep, fd-find, and fzf for developer productivity
3. Create __init__.py to expose main class
4. Create deps_devtools_snippet.Dockerfile:
   - Install ripgrep and fd-find via apt
   - Use Docker best practices
5. Add entry point to pyproject.toml
6. Add to EXTENSIONS_TO_TEST in test/test_extensions_generic.py
7. Add test_deps_devtools_extension method
8. Create test.sh in extension dir:
   - Check ripgrep, fd-find, fzf availability and basic usage
9. Make test.sh executable
10. Update README.md if needed
11. Commit only the contents of specs/01/deps-dev
