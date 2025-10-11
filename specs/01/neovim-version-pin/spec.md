# Fix nvim download

- Problem: `${NVIM_VERSION}` is empty in `nvim_snippet.Dockerfile`, so the build fetches `.../download//nvim...` and 404s.
- Goal: Pin nvim `v0.11.4` directly in the snippet to guarantee a valid download URL without templating.
- Success: Docker build succeeds and `pixi run ci` passes without manual input.
