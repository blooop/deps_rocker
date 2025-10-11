# nvim extension spec

- The nvim extension must install the Neovim binary (`nvim`) in the container.
- After installation, running `nvim --version` should work from any shell (i.e., `nvim` must be in `$PATH`).
- The test script should verify that `nvim` is available and functional.
- Use the latest stable release of Neovim unless otherwise specified.
- Installation should follow Docker best practices and use cache mounts for downloads.

# nvim extension fix plan

1. Update the nvim extension spec to clarify requirements (done).
2. Inspect and fix the nvim extension Dockerfile so that the `nvim` binary is installed to a directory in `$PATH` (e.g., `/usr/local/bin`).
3. Ensure the test script checks for `nvim` in `$PATH` and runs `nvim --version`.
4. Run `pixi run ci` to verify the fix.
5. Commit the changes if CI passes.
