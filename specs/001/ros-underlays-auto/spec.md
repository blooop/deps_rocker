# Automatic ROS underlay discovery

- Extend `ros_underlays.py` so `ros-underlays sync` recursively discovers newly imported `*.repos` manifests (e.g. `depends.repos`) and enqueues them before building, persisting the expanded manifest list for future runs.
- Ensure the build loop updates the on-disk manifest with any newly found underlays and keeps build order stable (dependencies first).
- Add a user-facing `ros-underlays rebuild` entrypoint that forces a rescan and rebuild from inside the container (exposed via alias/script) without needing a Docker rebuild.
