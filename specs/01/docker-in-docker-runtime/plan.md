# Plan: Docker-in-Docker Interactive Runtime

## Goal
Let the `docker_in_docker` extension produce containers where the Docker daemon is ready for interactive use without relying on extra init scripts.

## Steps
1. **Inspect Current Implementation**
   - Review `docker_in_docker.py`, `docker_in_docker_snippet.Dockerfile`, and related scripts to understand the existing init flow.
   - Identify why the detached rocker run hangs (likely waiting on `docker-init.sh`).
2. **Restructure Bootstrap Logic**
   - Inline the daemon setup into the Dockerfile snippet or entrypoint so the container can launch without copying auxiliary scripts.
   - Simplify runtime requirements while keeping privilege enforcement and data persistence mounts.
3. **Runtime Validation**
   - Update or add tests (e.g., in `test.sh`) to confirm `docker ps` works inside the container during CI.
   - Manually verify with `rockerc --docker-in-docker -f` if feasible.
4. **Polish and Document**
   - Ensure comments/docstrings explain the new flow and any limitations.
   - Update spec if clarifications arise.
5. **CI & Commit**
   - Run `pixi run ci`, fix failures, and commit once CI passes.
