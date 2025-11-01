# Test Package Cleanliness

- tests must no longer auto-generate throwaway ROS packages that conflict with existing fixtures
- after running `pixi run ci`, the git worktree stays clean with no leftover `test_package` artifacts
- update tests to rely on a tracked static fixture package under `test/test_package` (or similar) instead of generating content in the repo root
