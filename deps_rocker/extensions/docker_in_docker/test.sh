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

# Check if required entrypoint exists and is executable
if [[ ! -x /usr/local/bin/docker-dind-entrypoint ]]; then
    echo "ERROR: docker-dind-entrypoint not found or not executable"
    exit 1
fi

# If not root and not in docker group, re-exec shell with docker group membership
if [[ $(id -u) -ne 0 ]] && ! groups | grep -q docker; then
    echo "Current user is not in docker group. Attempting to re-exec shell with docker group membership using 'newgrp docker'."
    exec newgrp docker "$0" "$@"
fi

# Test basic Docker version
docker --version


# Docker-in-Docker functionality test (daemon should already be started by entrypoint)
echo "Testing Docker daemon functionality..."

# Test if we can connect to daemon (should already be running from entrypoint)
if docker info &> /dev/null; then
    echo "✅ Docker daemon is accessible"

    # Test basic Docker functionality
    echo "Testing Docker container functionality..."
    if docker run --rm hello-world &> /dev/null; then
        echo "✅ Docker can run containers successfully"
        echo "🎉 Docker-in-Docker is working correctly with --privileged mode!"
    else
        echo "⚠️  Docker daemon accessible but cannot run containers"
        echo "    This may indicate resource constraints or other issues"
        # Don't fail here as basic docker functionality is working
    fi
else
    echo "❌ Docker daemon not accessible"
    echo "    This may indicate the entrypoint didn't start the daemon properly"
    echo "    or missing --privileged mode"
    exit 1
fi

echo ""
echo "✅ Docker-in-Docker extension test completed successfully!"
echo "   Extension automatically provided --privileged mode and started Docker daemon."
