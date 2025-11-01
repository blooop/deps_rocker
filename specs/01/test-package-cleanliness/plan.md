# Plan

1. Inspect ros-related tests (e.g., `test/test_ros_jazzy_robustness.py`, `test/test_extensions_generic.py`) to locate logic that creates or copies `test_package` dynamically.
2. Trace where the generated package is written to the repository root and determine whether we can reference an existing static fixture or relocate output to a temp directory.
3. Refactor the tests and any supporting scripts to consume a tracked fixture package instead of creating new copies; ensure any necessary data lives under `test/test_package` (or similar) inside the repo.
4. Update helper utilities so they avoid writing to cwd or clean up after themselves; add safeguards in tests to assert the workspace remains clean.
5. Run `pixi run ci`, fix failing pieces, and repeat until the worktree stays clean and CI passes.
6. Commit the spec updates along with code changes once CI succeeds.
