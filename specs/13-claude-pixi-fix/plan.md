# Plan

- Compare current claude extension code, Dockerfile snippet, and tests to the working main branch to spot pixi migration regressions.
- Reproduce the failure via pixi tasks (e.g., `pixi run ci` or targeted tests) to confirm the breakage.
- Update the claude extension and tests to mirror main's behavior while remaining compatible with pixi-based workflows.
- Re-run pixi CI locally and iterate until tests pass; keep spec updated with any clarifications found.
