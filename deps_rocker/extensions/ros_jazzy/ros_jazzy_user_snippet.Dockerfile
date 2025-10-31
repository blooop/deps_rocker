# Generic user setup for any ROS workspace - avoid $HOME expansion issues

# Workspace environment variables with proper USERNAME expansion
ENV ROS_WORKSPACE_ROOT=/home/${USERNAME}/workspace
ENV ROS_INSTALL_BASE=/home/${USERNAME}/workspace/install
# Clone underlay dependencies from consolidated.repos using vcstool
RUN if [ -f /tmp/consolidated.repos ] && [ -s /tmp/consolidated.repos ]; then \
        mkdir -p /opt/ros/underlay/src && \
        vcs import --recursive /opt/ros/underlay/src < /tmp/consolidated.repos && \
        chown -R ${USERNAME}:${USERNAME} /opt/ros/underlay; \
    fi

# Build underlay workspace if dependencies exist
RUN if [ -d "/opt/ros/underlay/src" ] && [ "$(ls -A /opt/ros/underlay/src)" ]; then \
        /usr/local/bin/underlay_deps.sh && \
        /usr/local/bin/underlay_build.sh && \
        chown -R ${USERNAME}:${USERNAME} /opt/ros/underlay; \
    fi

# Copy colcon defaults configuration
COPY colcon-defaults.yaml /home/${USERNAME}/colcon-defaults.yaml
RUN chown ${USERNAME}:${USERNAME} /home/${USERNAME}/colcon-defaults.yaml

# Set up proper environment sourcing in bashrc - use absolute paths
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "if [ -f /opt/ros/underlay/install/setup.bash ]; then source /opt/ros/underlay/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc && \
    echo "export COLCON_DEFAULTS_FILE=/home/${USERNAME}/colcon-defaults.yaml" >> /home/${USERNAME}/.bashrc

# Install rosdeps for main workspace when it's mounted (generic approach)
RUN echo 'if [ ! -f "/home/${USERNAME}/.rosdeps_installed" ] && [ -d "$(pwd)" ] && find . -name "package.xml" -type f | head -1 | grep -q .; then' >> /home/${USERNAME}/.bashrc && \
    echo '  echo "Installing rosdep dependencies for main workspace..."' >> /home/${USERNAME}/.bashrc && \
    echo '  rosdep update' >> /home/${USERNAME}/.bashrc && \
    echo '  rosdep install --ignore-src --rosdistro jazzy --from-paths . -y -r || true' >> /home/${USERNAME}/.bashrc && \
    echo '  touch "/home/${USERNAME}/.rosdeps_installed"' >> /home/${USERNAME}/.bashrc && \
    echo 'fi' >> /home/${USERNAME}/.bashrc

# Set workspace directory (will be mounted at runtime)
WORKDIR /home/${USERNAME}/workspace
