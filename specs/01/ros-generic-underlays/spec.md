# ROS Generic Underlays

- Discover `*.repos` manifests in workspace and register them as chained ROS underlays.
- Follow `depends.repos` entries so dependent underlays auto-install.
- Ship a container script/alias to re-sync underlays without rebuilding the image.
