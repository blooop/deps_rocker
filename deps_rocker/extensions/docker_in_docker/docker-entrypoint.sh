#!/bin/bash
set -e

# Start Docker daemon in background if not already running
if ! pgrep dockerd > /dev/null 2>&1; then
    echo "Starting Docker daemon..."

    # Initialize Docker environment (from docker-init.sh)
    export container=docker

    if [ -d /sys/kernel/security ] && ! mountpoint -q /sys/kernel/security; then
        mount -t securityfs none /sys/kernel/security 2>/dev/null || echo "Warning: Could not mount /sys/kernel/security"
    fi

    # Only mount /tmp as tmpfs if it's empty (to avoid overwriting existing files)
    if ! mountpoint -q /tmp && [ -z "$(ls -A /tmp 2>/dev/null)" ]; then
        mount -t tmpfs none /tmp 2>/dev/null || echo "Warning: Could not mount /tmp"
    fi

    # cgroup v2: enable nesting
    if [ -f /sys/fs/cgroup/cgroup.controllers ]; then
        mkdir -p /sys/fs/cgroup/init 2>/dev/null || true
        xargs -rn1 < /sys/fs/cgroup/cgroup.procs > /sys/fs/cgroup/init/cgroup.procs 2>/dev/null || true
        sed -e 's/ / +/g' -e 's/^/+/' < /sys/fs/cgroup/cgroup.controllers \
            > /sys/fs/cgroup/cgroup.subtree_control 2>/dev/null || true
    fi

    # Start dockerd in background
    dockerd > /tmp/dockerd.log 2>&1 &

    # Wait for Docker daemon to be ready
    echo "Waiting for Docker daemon to start..."
    timeout=30
    while [ $timeout -gt 0 ]; do
        if docker info > /dev/null 2>&1; then
            echo "✅ Docker daemon is ready!"
            break
        fi
        sleep 1
        timeout=$((timeout - 1))
    done

    if [ $timeout -eq 0 ]; then
        echo "❌ Docker daemon failed to start within 30 seconds"
        echo "Check logs: cat /tmp/dockerd.log"
        exit 1
    fi
else
    echo "Docker daemon is already running"
fi

# Execute the original command
if [ $# -eq 0 ]; then
    # No command provided, start bash shell
    exec /bin/bash
else
    # Execute the provided command
    exec "$@"
fi
