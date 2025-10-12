# Docker-in-Docker Interactive Runtime

Ensure the `docker_in_docker` extension yields a rocker container whose interactive shell has a usable Docker CLI without manual init scripts.

- Bake Docker daemon bootstrap logic directly into the generated Dockerfile/entrypoint, avoiding separate helper scripts like `docker-init.sh`.
- Re-exec the entrypoint as root to launch `dockerd`, then drop back to the interactive user so `docker ps` succeeds.
- Confirm the container starts cleanly in interactive mode (e.g., `rockerc --docker-in-docker`) and `docker ps` works without extra steps.
- Preserve privileged-mode requirements and keep the implementation aligned with existing extension patterns.
