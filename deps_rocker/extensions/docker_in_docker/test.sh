#!/bin/bash
set -e

echo "Testing docker_in_docker installation..."

# Check if Docker commands are available
if ! command -v docker &> /dev/null; then
    echo "ERROR: docker command not found"
    exit 1
fi

if ! command -v dockerd &> /dev/null; then
    echo "ERROR: dockerd command not found"
    exit 1
fi

# Check if Docker Compose plugin is available
if ! docker compose version &> /dev/null; then
    echo "ERROR: docker compose plugin not found"
    exit 1
fi

# Check if Docker Buildx plugin is available
if ! docker buildx version &> /dev/null; then
    echo "ERROR: docker buildx plugin not found"
    exit 1
fi

# Check if required scripts exist and are executable
if [[ ! -x /usr/local/bin/docker-entrypoint.sh ]]; then
    echo "ERROR: docker-entrypoint.sh not found or not executable"
    exit 1
fi

if [[ ! -x /usr/local/share/docker-init.sh ]]; then
    echo "ERROR: docker-init.sh not found or not executable"
    exit 1
fi

# Check if user is in docker group (if not root)
if [[ $(id -u) -ne 0 ]] && ! groups | grep -q docker; then
    echo "ERROR: Current user is not in docker group"
    exit 1
fi

# Test basic Docker version
docker --version

echo "NOTE: To use Docker-in-Docker, the container must be run with --privileged flag"
echo "Example: rocker --docker-in-docker --privileged ubuntu:22.04"

echo "docker_in_docker extension test completed successfully!"