

# Create unified workspace architecture directory structure
RUN mkdir -p /home/@(name)/underlay/src /home/@(name)/underlay/build /home/@(name)/underlay/install /home/@(name)/underlay/log && \
    mkdir -p /home/@(name)/overlay/src /home/@(name)/overlay/build /home/@(name)/overlay/install /home/@(name)/overlay/log


# Set up unified workspace environment variables
# Docker ENV is sufficient for most use cases - ROS tools and build systems inherit container environment
ENV ROS_UNDERLAY_ROOT="/home/@(name)/underlay" \
    ROS_UNDERLAY_PATH="/home/@(name)/underlay/src" \
    ROS_UNDERLAY_BUILD="/home/@(name)/underlay/build" \
    ROS_UNDERLAY_INSTALL="/home/@(name)/underlay/install" \
    ROS_OVERLAY_ROOT="/home/@(name)/overlay" \
    ROS_WORKSPACE_ROOT="/home/@(name)/overlay" \
    ROS_BUILD_BASE="/home/@(name)/overlay/build" \
    ROS_INSTALL_BASE="/home/@(name)/overlay/install" \
    ROS_LOG_BASE="/home/@(name)/overlay/log"

# Copy consolidated repos file if it exists
COPY consolidated.repos /tmp/consolidated.repos

# Clone underlay dependencies from consolidated.repos using vcstool
RUN if [ -f /tmp/consolidated.repos ] && [ -s /tmp/consolidated.repos ]; then \
        mkdir -p /home/@(name)/underlay/src && \
        vcs import --recursive /home/@(name)/underlay/src < /tmp/consolidated.repos && \
        chown -R @(name):@(name) /home/@(name)/underlay; \
    fi

# Build underlay workspace if dependencies exist
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    --mount=type=cache,target=/home/@(name)/.cache/pip,id=pip-cache \
    if [ -d "/home/@(name)/underlay/src" ] && [ "$(ls -A /home/@(name)/underlay/src)" ]; then \
        rm -rf /home/@(name)/underlay/build /home/@(name)/underlay/install && \
        mkdir -p /home/@(name)/underlay/build /home/@(name)/underlay/install && \
        /usr/local/bin/underlay_deps.sh && \
        /usr/local/bin/underlay_build.sh && \
        chown -R @(name):@(name) /home/@(name)/underlay; \
    fi


COPY --chown=@(name):@(name) colcon-defaults.yaml /home/@(name)/colcon-defaults.yaml

# Set up proper environment sourcing in bashrc - unified workspace architecture
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/@(name)/.bashrc && \
    echo "if [ -f /home/@(name)/underlay/install/setup.bash ]; then source /home/@(name)/underlay/install/setup.bash; fi" >> /home/@(name)/.bashrc && \
    echo "if [ -f /home/@(name)/overlay/install/setup.bash ]; then source /home/@(name)/overlay/install/setup.bash; fi" >> /home/@(name)/.bashrc && \
    echo "export COLCON_DEFAULTS_FILE=/home/@(name)/colcon-defaults.yaml" >> /home/@(name)/.bashrc

# Set workspace directory to overlay (user code workspace)
WORKDIR /home/@(name)/overlay

# Switch to the user context for final container execution
USER @(name)
