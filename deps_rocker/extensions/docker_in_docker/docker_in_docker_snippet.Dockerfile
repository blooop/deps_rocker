# Docker-in-Docker Extension
#
# 🚨 CRITICAL REQUIREMENT: This extension REQUIRES --privileged mode to function!
#
# Docker-in-Docker needs kernel access and system capabilities that are only
# available with privileged containers. Without --privileged, the Docker daemon
# cannot start and this extension will not work.
#
# AUTOMATIC rocker arguments (provided by extension):
#   --privileged                              (REQUIRED - Docker daemon cannot start without this)
#   --volume /var/lib/docker                  (Docker data persistence within container)
#
# SIMPLE USAGE (extension automatically provides all required arguments):
#   rocker --docker-in-docker ubuntu:22.04
#
# NOTE: This is true Docker-in-Docker - runs a separate Docker daemon inside the container.
# This is different from "docker-out-of-docker" which would mount the host's socket.
#
# Advanced users only (alternative to --privileged):
#   --cap-add SYS_ADMIN --cap-add DAC_READ_SEARCH --security-opt apparmor:unconfined
#   --cgroup-parent docker.slice --cgroupns private
#
# ⚠️  WARNING: Running without --privileged will result in test failures!

# Install prerequisites for Docker CE and runtime helpers
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    gosu \
    lsb-release \
    sudo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Add Docker's official GPG key
RUN install -m 0755 -d /etc/apt/keyrings \
    && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg \
    && chmod a+r /etc/apt/keyrings/docker.gpg

# Add Docker repository
RUN ARCH=$(dpkg --print-architecture) \
    && echo "deb [arch=${ARCH} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create Docker-in-Docker entrypoint script inline so no helper files are required
RUN cat <<'EOF' >/usr/local/bin/docker-dind-entrypoint && chmod +x /usr/local/bin/docker-dind-entrypoint
#!/bin/sh
set -eu

log() {
    printf '[docker-in-docker] %s\n' "$*"
}

ORIGINAL_USER="${USER:-}"
if [ -z "$ORIGINAL_USER" ]; then
    ORIGINAL_USER="$(id -un 2>/dev/null || echo root)"
fi

if [ "$(id -u)" -ne 0 ]; then
    exec sudo -E DIND_TARGET_USER="$ORIGINAL_USER" "$0" "$@@"
fi

TARGET_USER="${DIND_TARGET_USER:-$ORIGINAL_USER}"
unset DIND_TARGET_USER

if [ -z "$TARGET_USER" ] || ! id -u "$TARGET_USER" >/dev/null 2>&1; then
    TARGET_USER="root"
fi

log "starting docker daemon for user '$TARGET_USER'"

if [ -f /var/run/docker.pid ] && ! pgrep -x dockerd >/dev/null 2>&1; then
    rm -f /var/run/docker.pid
fi

mkdir -p /var/run/docker /var/lib/docker

if [ -d /sys/kernel/security ] && ! mountpoint -q /sys/kernel/security; then
    if ! mount -t securityfs none /sys/kernel/security 2>/dev/null; then
        log "warning: failed to mount /sys/kernel/security (AppArmor may be disabled)"
    fi
fi

if ! mountpoint -q /tmp; then
    mount -t tmpfs none /tmp || true
fi

if [ -f /sys/fs/cgroup/cgroup.controllers ]; then
    mkdir -p /sys/fs/cgroup/init
    xargs -rn1 </sys/fs/cgroup/cgroup.procs >/sys/fs/cgroup/init/cgroup.procs 2>/dev/null || true
    sed -e 's/ / +/g' -e 's/^/+/' </sys/fs/cgroup/cgroup.controllers >/sys/fs/cgroup/cgroup.subtree_control
fi

if ! getent group docker >/dev/null 2>&1; then
    groupadd -f docker
fi

if [ "$TARGET_USER" != "root" ] && id -u "$TARGET_USER" >/dev/null 2>&1; then
    usermod -aG docker "$TARGET_USER" || true
fi

if ! pgrep -x dockerd >/dev/null 2>&1; then
    dockerd --host=unix:///var/run/docker.sock \
            --exec-root=/var/run/docker \
            --data-root=/var/lib/docker \
            > /var/log/dockerd.log 2>&1 &
    DIND_DOCKERD_PID=$!
else
    DIND_DOCKERD_PID=""
fi

TRIES=0
while [ ! -S /var/run/docker.sock ]; do
    if [ -n "$DIND_DOCKERD_PID" ] && ! kill -0 "$DIND_DOCKERD_PID" 2>/dev/null; then
        log "dockerd exited unexpectedly; see /var/log/dockerd.log"
        exit 1
    fi
    if [ "$TRIES" -ge 120 ]; then
        log "timeout waiting for /var/run/docker.sock"
        exit 1
    fi
    sleep 0.5
    TRIES=$((TRIES + 1))
done

chown root:docker /var/run/docker.sock || true
chmod 660 /var/run/docker.sock || true

if [ "$#" -eq 0 ]; then
    set -- /bin/bash
fi

if [ "$TARGET_USER" != "root" ]; then
    exec gosu "$TARGET_USER" "$@@"
else
    exec "$@@"
fi
EOF

ENTRYPOINT ["/usr/local/bin/docker-dind-entrypoint"]
