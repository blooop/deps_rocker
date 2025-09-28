# Docker-in-Docker Extension
#
# IMPORTANT: This extension requires the container to run with specific privileges and mounts:
#
# Required rocker arguments:
#   --privileged          (required for Docker daemon to start)
#   --volume /var/lib/docker  (optional: persist Docker data)
#
# Alternative minimal privileges (instead of --privileged):
#   --cap-add SYS_ADMIN
#   --cap-add DAC_READ_SEARCH
#   --security-opt apparmor:unconfined
#   --cgroup-parent docker.slice
#   --cgroupns private
#
# Example usage:
#   rocker --docker-in-docker --privileged ubuntu:22.04

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