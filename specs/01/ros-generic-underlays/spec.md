# ROS Generic Underlays

- Discover workspace `*.repos` manifests during build and fold them into ROS underlays.
- Resolve `depends.repos` chains recursively so secondary manifests are installed automatically.
- Provide in-container `ros-underlays` helper (script+alias) that rebuilds the underlays without rebuilding the image.
