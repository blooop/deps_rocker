# Implementation Plan

1) Baseline the diff and targets
- Compare branch against main to list packages/extensions still using apt or custom installs that should move to pixi.
- Note related snippets, tests, and entry points impacted by the migration.

2) Validate pixi task wiring
- Inspect `pyproject.toml` pixi tasks to ensure they cover the migrated packages and CI entrypoints.
- Adjust tasks or dependencies as needed to match the port.

3) Run CI and fix failures
- Execute `pixi run ci` to surface regressions from the pixi migration.
- Update extensions/snippets/tests to rely on pixi installs; fix any path/env or version issues uncovered by CI.

4) Re-run CI to green
- Re-run `pixi run ci` until it passes.
- Keep changes minimal and aligned with the pixi adoption goal.

5) Polish
- Update comments/docs to reference pixi where relevant.
- Prepare final changes for commit once CI is green.
