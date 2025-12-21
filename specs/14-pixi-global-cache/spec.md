# Fix pixi globals and cache footprint
Category: build-optimization

- Stop baking pixi global installs into build layers; ship a manifest and sync globals at runtime instead.
- Ensure manifest lists all required global tools with constraints and is copied into the image.
- Add a runtime entrypoint that runs `pixi global sync --manifest` against mounted caches so first run installs, later runs are instant.
- Build the manifest from `pixi_global_packages` contributed by enabled extensions to avoid clobbering tools when installs happen separately.
- Keep layers small by avoiding build-time pixi downloads; rely on host/volume caches.
- Validate images still surface expected tools after first run and CI passes without disk blowups.
