# Generic ROS 2 Jazzy setup - works with any ROS repository
ENV DEBIAN_FRONTEND=noninteractive

# Prevent services from starting during package installation
# This is critical for Docker containers as they don't run systemd as init
RUN echo '#!/bin/sh\nexit 101' > /usr/sbin/policy-rc.d && chmod +x /usr/sbin/policy-rc.d

# Configure apt to wait for locks instead of failing immediately
# This prevents hangs/failures when multiple processes try to use apt simultaneously
RUN echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries && \
    echo 'DPkg::Lock::Timeout "120";' > /etc/apt/apt.conf.d/80-dpkg-lock

# Install ROS2 repository and key
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN --mount=type=cache,target=/var/cache/apt,sharing=private,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=private,id=apt-lists \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-core \
    python3-rosdep

# Install colcon tooling, numpy, and lark via pip
RUN --mount=type=cache,target=/root/.cache/pip,id=pip-cache \
    pip install vcs2l colcon-common-extensions colcon-defaults colcon-runner colcon-clean numpy lark --break-system-packages

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
  else \
    echo "rosdep already initialized, skipping init"; \
  fi

# Create cache directories for BuildKit mounts
RUN mkdir -p /root/.cache/vcs-repos && \
    chmod 755 /root/.cache/vcs-repos

# Copy scripts and make them executable
COPY underlay_deps.sh underlay_build.sh install_rosdeps.sh /usr/local/bin/
COPY rosdep_underlay.sh rosdep_overlay.sh build_underlay.sh update_repos.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/underlay_deps.sh /usr/local/bin/underlay_build.sh /usr/local/bin/install_rosdeps.sh \
             /usr/local/bin/rosdep_underlay.sh /usr/local/bin/rosdep_overlay.sh \
             /usr/local/bin/build_underlay.sh /usr/local/bin/update_repos.sh

# Create underlay and overlay workspace directory structure
RUN mkdir -p /home/@(name)/underlay/src /home/@(name)/underlay/build /home/@(name)/underlay/install && \
    mkdir -p /home/@(name)/overlay/build /home/@(name)/overlay/install && \
    chown -R @(name):@(name) /home/@(name)/underlay /home/@(name)/overlay

# Create user-accessible cache directories for BuildKit mounts
RUN mkdir -p /home/@(name)/.cache/pip && \
    chown -R @(name):@(name) /home/@(name)/.cache

# Copy consolidated repos file (always exists, may be empty)
COPY consolidated.repos /tmp/consolidated.repos

# Import repositories from consolidated.repos into underlay workspace
# Use cache mount for repository cloning to speed up builds
RUN --mount=type=cache,target=/root/.cache/vcs-repos,id=vcs-repos-cache \
    --mount=type=cache,target=/home/@(name)/.cache/pip,id=pip-cache-@(name) \
    su - @(name) -c 'update_repos.sh' || echo "No repositories to import"
