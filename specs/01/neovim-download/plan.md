1. Inspect neovim-related files for how `NEOVIM_VERSION` should be set and ensure consistent propagation into Docker snippets and extension logic.
2. Update the neovim Docker snippet or extension to supply a concrete version, adjust extraction paths, and add guardrails for cache re-use.
3. Rename duplicate build stages and resolve undefined `$NODE_VERSION` by wiring the value from the npm extension configuration.
4. Convert legacy `ENV key value` lines to `ENV key=value` format in the impacted Docker snippet(s).
5. Re-run `pixi run ci`, iterate on fixes, and commit once the pipeline succeeds.

