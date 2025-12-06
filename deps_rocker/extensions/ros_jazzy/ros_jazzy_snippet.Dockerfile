# ROS 2 Jazzy setup using robotstack via pixi
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS Jazzy and colcon from robotstack
RUN pixi global install ros-jazzy-desktop colcon-common-extensions

# Set up ROS environment variables
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Create workspace directories
RUN mkdir -p /home/@(name)/underlay/src /home/@(name)/underlay/build /home/@(name)/underlay/install && \
    mkdir -p /home/@(name)/overlay/build /home/@(name)/overlay/install && \
    chown -R @(name):@(name) /home/@(name)/underlay /home/@(name)/overlay

# Create user-accessible cache directories
RUN mkdir -p /home/@(name)/.cache/pip && \
    chown -R @(name):@(name) /home/@(name)/.cache

# Copy scripts and make them executable (if they exist)
COPY --chown=@(name):@(name) underlay_deps.sh underlay_build.sh install_rosdeps.sh /usr/local/bin/ 2>/dev/null || true
COPY --chown=@(name):@(name) rosdep_underlay.sh rosdep_overlay.sh build_underlay.sh update_repos.sh /usr/local/bin/ 2>/dev/null || true
RUN chmod +x /usr/local/bin/*.sh 2>/dev/null || true

# Copy consolidated repos file if it exists
COPY --chown=@(name):@(name) consolidated.repos /tmp/consolidated.repos 2>/dev/null || true

# Import repositories from consolidated.repos into underlay workspace (if file exists)
RUN --mount=type=cache,target=/root/.cache/vcs-repos,id=vcs-repos-cache \
    --mount=type=cache,target=/home/@(name)/.cache/pip,id=pip-cache-@(name) \
    if [ -f /tmp/consolidated.repos ]; then \
        su - @(name) -c 'update_repos.sh' || echo "No repositories to import"; \
    fi
