# Plan

1. Map current `ros_underlays.py` flow (manifest loading, sync command, aggregator updates) and note where newly imported manifests can be discovered.
2. Implement recursive discovery queue: after importing a manifest, rescan its source tree for fresh `*.repos`, persist them, and ensure dependencies build before dependents; update manifest persistence accordingly.
3. Extend CLI with a `rebuild` action that triggers a forced rescan/rebuild, expose it via alias/script in the Docker snippet, and exercise existing test coverage or add targeted tests if necessary.
