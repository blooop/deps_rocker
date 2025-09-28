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

# Install prerequisites
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    lsb-release \
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

# Create docker group and add current user
RUN groupadd -f docker

# Install Docker-in-Docker initialization script
COPY docker-init.sh /usr/local/share/docker-init.sh
RUN chmod +x /usr/local/share/docker-init.sh

# Create entrypoint script
COPY docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Set entrypoint to automatically start Docker daemon
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
