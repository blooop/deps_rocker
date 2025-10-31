# Create unified workspace architecture directory structure
RUN mkdir -p /home/@(name)/underlay/src /home/@(name)/underlay/build /home/@(name)/underlay/install /home/@(name)/underlay/log && \
    mkdir -p /home/@(name)/overlay/src /home/@(name)/overlay/build /home/@(name)/overlay/install /home/@(name)/overlay/log && \
    chown -R @(name):@(name) /home/@(name)/underlay /home/@(name)/overlay

# Set up unified workspace environment variables both as ENV commands and in bashrc
# ENV commands ensure availability in all shell contexts, bashrc for interactive shells
ENV ROS_UNDERLAY_ROOT="/home/@(name)/underlay" \
    ROS_UNDERLAY_PATH="/home/@(name)/underlay/src" \
    ROS_UNDERLAY_BUILD="/home/@(name)/underlay/build" \
    ROS_UNDERLAY_INSTALL="/home/@(name)/underlay/install" \
    ROS_OVERLAY_ROOT="/home/@(name)/overlay" \
    ROS_WORKSPACE_ROOT="/home/@(name)/overlay" \
    ROS_BUILD_BASE="/home/@(name)/overlay/build" \
    ROS_INSTALL_BASE="/home/@(name)/overlay/install" \
    ROS_LOG_BASE="/home/@(name)/overlay/log"

RUN echo "# ROS Unified Workspace Architecture Environment Variables" >> /home/@(name)/.bashrc && \
    echo "export ROS_UNDERLAY_ROOT=\$HOME/underlay" >> /home/@(name)/.bashrc && \
    echo "export ROS_UNDERLAY_PATH=\$HOME/underlay/src" >> /home/@(name)/.bashrc && \
    echo "export ROS_UNDERLAY_BUILD=\$HOME/underlay/build" >> /home/@(name)/.bashrc && \
    echo "export ROS_UNDERLAY_INSTALL=\$HOME/underlay/install" >> /home/@(name)/.bashrc && \
    echo "export ROS_OVERLAY_ROOT=\$HOME/overlay" >> /home/@(name)/.bashrc && \
    echo "export ROS_WORKSPACE_ROOT=\$HOME/overlay" >> /home/@(name)/.bashrc && \
    echo "export ROS_BUILD_BASE=\$HOME/overlay/build" >> /home/@(name)/.bashrc && \
    echo "export ROS_INSTALL_BASE=\$HOME/overlay/install" >> /home/@(name)/.bashrc && \
    echo "export ROS_LOG_BASE=\$HOME/overlay/log" >> /home/@(name)/.bashrc
# Clone underlay dependencies from consolidated.repos using vcstool
RUN if [ -f /tmp/consolidated.repos ] && [ -s /tmp/consolidated.repos ]; then \
        vcs import --recursive /home/@(name)/underlay/src < /tmp/consolidated.repos && \
        chown -R @(name):@(name) /home/@(name)/underlay; \
    fi

# Build underlay workspace if dependencies exist
RUN if [ -d "/home/@(name)/underlay/src" ] && [ "$(ls -A /home/@(name)/underlay/src)" ]; then \
        /usr/local/bin/underlay_deps.sh && \
        /usr/local/bin/underlay_build.sh && \
        chown -R @(name):@(name) /home/@(name)/underlay; \
    fi

# Copy colcon defaults configuration
COPY --chown=@(name):@(name) colcon-defaults.yaml /home/@(name)/colcon-defaults.yaml

# Set up proper environment sourcing in bashrc - unified workspace architecture
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/@(name)/.bashrc && \
    echo "if [ -f /home/@(name)/underlay/install/setup.bash ]; then source /home/@(name)/underlay/install/setup.bash; fi" >> /home/@(name)/.bashrc && \
    echo "if [ -f /home/@(name)/overlay/install/setup.bash ]; then source /home/@(name)/overlay/install/setup.bash; fi" >> /home/@(name)/.bashrc && \
    echo "export COLCON_DEFAULTS_FILE=/home/@(name)/colcon-defaults.yaml" >> /home/@(name)/.bashrc

# Install rosdeps for main workspace when it's mounted (generic approach)
RUN echo 'if [ ! -f "/home/@(name)/.rosdeps_installed" ] && [ -d "$(pwd)" ] && find . -name "package.xml" -type f | head -1 | grep -q .; then' >> /home/@(name)/.bashrc && \
    echo '  echo "Installing rosdep dependencies for main workspace..."' >> /home/@(name)/.bashrc && \
    echo '  rosdep update' >> /home/@(name)/.bashrc && \
    echo '  rosdep install --ignore-src --rosdistro jazzy --from-paths . -y -r || true' >> /home/@(name)/.bashrc && \
    echo '  touch "/home/@(name)/.rosdeps_installed"' >> /home/@(name)/.bashrc && \
    echo 'fi' >> /home/@(name)/.bashrc

# Set workspace directory to overlay (user code workspace)
WORKDIR /home/@(name)/overlay

# Switch to the user context for final container execution
USER @(name)
