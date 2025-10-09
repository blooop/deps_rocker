# nvim extension fix plan

1. Update the nvim extension spec to clarify requirements (done).
2. Inspect and fix the nvim extension Dockerfile so that the `nvim` binary is installed to a directory in `$PATH` (e.g., `/usr/local/bin`).
3. Ensure the test script checks for `nvim` in `$PATH` and runs `nvim --version`.
4. Run `pixi run ci` to verify the fix.
5. Commit the changes if CI passes.
