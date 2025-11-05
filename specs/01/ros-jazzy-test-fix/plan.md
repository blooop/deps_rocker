# Plan

1. Run `pixi r test-extension ros_jazzy` to capture the current failure details.
2. Trace the failure to the relevant ros_jazzy code, Docker snippets, or test fixtures and decide on a corrective approach.
3. Implement the fix, keeping the extension checklist updated as needed.
4. Re-run the targeted test to confirm the issue is resolved.
5. Execute `pixi run ci` for full regression coverage and address any remaining breakage.
6. Prepare documentation or spec updates if new behaviors emerge.
