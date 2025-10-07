# Plan: CI Health

1. Capture current failures by running `pixi run ci` and saving logs for reference.
2. Triage the failing steps to identify root causes and prioritize fixes.
3. Implement code or configuration changes to address each failure, updating tests when needed.
4. Re-run `pixi run ci` after each fix to confirm progress and iterate until the command succeeds.
5. Stage and commit the validated changes, preserving a clear history.
