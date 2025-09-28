#!/bin/bash
set -e

# Set up signal handlers for graceful shutdown
cleanup() {
    echo "Shutting down Docker daemon..."
    if [ -n "$DOCKERD_PID" ] && kill -0 $DOCKERD_PID 2>/dev/null; then
        kill $DOCKERD_PID
        wait $DOCKERD_PID 2>/dev/null || true
    fi
    exit 0
}
trap cleanup TERM INT

# Start Docker daemon in background if not already running
if ! pgrep dockerd > /dev/null 2>&1; then
    echo "Starting Docker daemon..."

    # Check if we're running as root, if not, use sudo
    if [ "$(id -u)" -ne 0 ]; then
        echo "Current user is not root (UID: $(id -u)), using sudo to start dockerd"
        DOCKERD_CMD="sudo"
    else
        echo "Running as root, starting dockerd directly"
        DOCKERD_CMD=""
    fi

    # Initialize Docker environment as root
    if [ -n "$DOCKERD_CMD" ]; then
        sudo bash -c '
            export container=docker

            if [ -d /sys/kernel/security ] && ! mountpoint -q /sys/kernel/security; then
                mount -t securityfs none /sys/kernel/security 2>/dev/null || echo "Warning: Could not mount /sys/kernel/security"
            fi

            # cgroup v2: enable nesting
            if [ -f /sys/fs/cgroup/cgroup.controllers ]; then
                mkdir -p /sys/fs/cgroup/init 2>/dev/null || true
                xargs -rn1 < /sys/fs/cgroup/cgroup.procs > /sys/fs/cgroup/init/cgroup.procs 2>/dev/null || true
                sed -e "s/ / +/g" -e "s/^/+/" < /sys/fs/cgroup/cgroup.controllers \
                    > /sys/fs/cgroup/cgroup.subtree_control 2>/dev/null || true
            fi
        '
    else
        # Initialize Docker environment (from docker-init.sh)
        export container=docker

        if [ -d /sys/kernel/security ] && ! mountpoint -q /sys/kernel/security; then
            mount -t securityfs none /sys/kernel/security 2>/dev/null || echo "Warning: Could not mount /sys/kernel/security"
        fi

        # cgroup v2: enable nesting
        if [ -f /sys/fs/cgroup/cgroup.controllers ]; then
            mkdir -p /sys/fs/cgroup/init 2>/dev/null || true
            xargs -rn1 < /sys/fs/cgroup/cgroup.procs > /sys/fs/cgroup/init/cgroup.procs 2>/dev/null || true
            sed -e 's/ / +/g' -e 's/^/+/' < /sys/fs/cgroup/cgroup.controllers \
                > /sys/fs/cgroup/cgroup.subtree_control 2>/dev/null || true
        fi
    fi

    # Skip mounting /tmp as tmpfs in complex environments to avoid conflicts
    echo "Skipping /tmp mount to avoid conflicts with volume mounts"

    # Start dockerd in background with more verbose logging and better configuration
    echo "Starting dockerd..."

    # Use proper data-root and better default configuration
    DOCKERD_ARGS="--data-root=/var/lib/docker --log-level=info --storage-driver=overlay2"

    if [ -n "$DOCKERD_CMD" ]; then
        sudo dockerd $DOCKERD_ARGS > /tmp/dockerd.log 2>&1 &
    else
        dockerd $DOCKERD_ARGS > /tmp/dockerd.log 2>&1 &
    fi
    DOCKERD_PID=$!

    # Give dockerd a moment to initialize
    sleep 3

    # Wait for Docker daemon to be ready with progress indicators
    echo "Waiting for Docker daemon to start..."
    timeout=90  # Increased timeout for complex environments
    counter=0
    while [ $timeout -gt 0 ]; do
        # Check if docker socket is available and responding
        if [ -S /var/run/docker.sock ] && docker version > /dev/null 2>&1; then
            echo "✅ Docker daemon is ready!"
            break
        fi

        # Show progress every 5 seconds
        if [ $((counter % 5)) -eq 0 ]; then
            echo "Still waiting... ($timeout seconds remaining)"
            # Check if dockerd process is still running
            if ! kill -0 $DOCKERD_PID 2>/dev/null; then
                echo "❌ Docker daemon process died!"
                echo "Last 20 lines of dockerd.log:"
                tail -20 /tmp/dockerd.log 2>/dev/null || echo "No log available"
                exit 1
            fi
            # Show socket status
            if [ -S /var/run/docker.sock ]; then
                echo "   Socket exists, testing connectivity..."
            else
                echo "   Waiting for socket creation..."
            fi
        fi

        sleep 1
        timeout=$((timeout - 1))
        counter=$((counter + 1))
    done

    if [ $timeout -eq 0 ]; then
        echo "❌ Docker daemon failed to start within 90 seconds"
        echo "Docker daemon process status: $(kill -0 $DOCKERD_PID 2>/dev/null && echo 'running' || echo 'not running')"
        echo "Socket status: $([ -S /var/run/docker.sock ] && echo 'exists' || echo 'missing')"
        echo "Last 50 lines of dockerd.log:"
        tail -50 /tmp/dockerd.log 2>/dev/null || echo "No log available"
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
