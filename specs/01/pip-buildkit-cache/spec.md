# Pip BuildKit Cache Mounts

## Objective
Add BuildKit cache mounts to all Dockerfiles that use `pip install` to speed up builds and reduce network traffic.

## Scope
Update the following files to use `RUN --mount=type=cache,target=/root/.cache/pip`:
- `deps_rocker/templates/vcstool_snippet.Dockerfile`
- `deps_rocker/extensions/vcstool/vcstool_snippet.Dockerfile`
- `deps_rocker/extensions/ros_jazzy/ros_jazzy_snippet.Dockerfile`
- `deps_rocker/extensions/isaac_sim/isaacsim_snippet.Dockerfile`

## Implementation
Replace `RUN pip install ...` with `RUN --mount=type=cache,target=/root/.cache/pip pip install ...` to enable BuildKit caching of pip downloads.

## Benefits
- Faster rebuilds when pip packages are already cached
- Reduced network bandwidth usage
- More efficient CI/CD pipelines
