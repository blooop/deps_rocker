# Fix Neovim download

- Problem: `${NEOVIM_VERSION}` is empty in `neovim_snippet.Dockerfile`, so the build fetches `.../download//nvim...` and 404s.
- Goal: Pin Neovim `v0.11.4` directly in the snippet to guarantee a valid download URL without templating.
- Success: Docker build succeeds and `pixi run ci` passes without manual input.
