# Docker-in-Docker Interactive Runtime

Ensure the `docker_in_docker` extension yields a rocker container whose interactive shell has a usable Docker CLI without manual init scripts.

- Bake all Docker daemon bootstrap logic directly into the generated Dockerfile/entrypoint, avoiding separate helper scripts.
- Confirm the container starts cleanly in interactive mode (e.g., `rockerc --docker-in-docker`) and `docker ps` succeeds.
- Preserve privileged-mode requirements and keep the implementation aligned with existing extension patterns.
