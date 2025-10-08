# Plan

1. Inspect `deps_rocker/extensions/neovim/neovim_snippet.Dockerfile` to confirm how `NEOVIM_VERSION` is currently provided.
2. Choose a supported release tag (e.g. `stable` or a pinned version like `v0.9.5`) that has a matching GitHub tarball.
3. Update the Dockerfile to set the version explicitly and ensure the download path matches the archive name.
4. Verify related stages or tests do not assume an external version arg; adjust if needed.
5. Run `pixi run ci`, address any failures, and keep iterating until green.
6. Commit the changes once CI passes.
