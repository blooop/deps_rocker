# Test Package Cleanliness

- tests must no longer auto-generate throwaway ROS packages that conflict with existing fixtures
- after running `pixi run ci`, the git worktree stays clean with no leftover `test_package` artifacts
- reuse or create deterministic fixtures so extension tests still validate expected behaviour
