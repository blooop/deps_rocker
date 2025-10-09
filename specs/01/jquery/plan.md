# Plan for jquery extension

1. Implement extension class `Jquery` inheriting `SimpleRockerExtension` with name `jquery`.
2. Provide Docker snippet file `jquery_snippet.Dockerfile` performing apt-get install jq with cleanup.
3. Add `__init__.py` exporting class.
4. Add test.sh verifying `command -v jq` and running a trivial jq filter.
5. Register entry point in pyproject `[project.entry-points."rocker.extensions"]`.
6. Add `jquery` to `EXTENSIONS_TO_TEST` list and individual test method in `test_extensions_generic.py`.
7. Run `pixi run ci` and fix any style/lint/test issues.
8. Commit spec + implementation only after CI passes.

Assumptions:
- No existing extension requires jq; thus no dependency chain.
- Accept Ubuntu repo version of jq.

Risks / Edge Cases:
- Naming confusion: extension called jquery but installs jq. Documented in spec.
- apt cache cleanup to keep layer small.
