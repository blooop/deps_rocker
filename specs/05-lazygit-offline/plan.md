## Steps
- Inspect lazygit builder snippet and current version pinning; remove reliance on GitHub “latest” API.
- Change builder to use the pinned LAZYGIT_VERSION from empy args, with retrying `curl -fL` and cache reuse.
- Run `pixi run ci`; fix any remaining failures and commit once CI passes.
