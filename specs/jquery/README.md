# jQuery Extension Spec

Goal: Provide a simple extension named `jquery` which installs the `jq` command-line JSON processor via apt for use in other image build steps. (Name chosen per request even though it provides `jq`).

Scope:
- New rocker extension `jquery` depending on no other extensions.
- Installs apt package `jq` using non-interactive, minimal layer pattern.
- Adds entry point so `--jquery` flag works.
- Provides test script verifying `jq` presence and basic functionality.

Non-goals: Managing JavaScript jQuery library, version pinning for jq beyond apt default.

Outputs: extension code, Docker snippet, test.sh, updated pyproject entry point, test list inclusion.

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
