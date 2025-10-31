# Set environment variables properly by expanding $HOME in a shell command and setting them
RUN ROS_WS_ROOT="$HOME/ros_ws" && \
    echo "export ROS_WORKSPACE_ROOT=$ROS_WS_ROOT" >> ~/.bashrc && \
    echo "export ROS_UNDERLAY_PATH=$ROS_WS_ROOT/underlay" >> ~/.bashrc && \
    echo "export ROS_UNDERLAY_BUILD=$ROS_WS_ROOT/underlay_build" >> ~/.bashrc && \
    echo "export ROS_UNDERLAY_INSTALL=$ROS_WS_ROOT/underlay_install" >> ~/.bashrc && \
    echo "export ROS_BUILD_BASE=$ROS_WS_ROOT/build" >> ~/.bashrc && \
    echo "export ROS_INSTALL_BASE=$ROS_WS_ROOT/install" >> ~/.bashrc && \
    echo "export ROS_LOG_BASE=$ROS_WS_ROOT/log" >> ~/.bashrc && \
    echo "export COLCON_LOG_PATH=$ROS_WS_ROOT/log" >> ~/.bashrc

# Create ROS workspace directories as user
RUN export ROS_WORKSPACE_ROOT="$HOME/ros_ws" && \
    export ROS_UNDERLAY_PATH="$HOME/ros_ws/underlay" && \
    export ROS_UNDERLAY_BUILD="$HOME/ros_ws/underlay_build" && \
    export ROS_UNDERLAY_INSTALL="$HOME/ros_ws/underlay_install" && \
    export ROS_BUILD_BASE="$HOME/ros_ws/build" && \
    export ROS_INSTALL_BASE="$HOME/ros_ws/install" && \
    export ROS_LOG_BASE="$HOME/ros_ws/log" && \
    mkdir -p "$ROS_UNDERLAY_PATH" "$ROS_UNDERLAY_BUILD" "$ROS_UNDERLAY_INSTALL" \
             "$ROS_BUILD_BASE" "$ROS_INSTALL_BASE" "$ROS_LOG_BASE"

# Import the consolidated depends.repos manifest to underlay as user
RUN export ROS_UNDERLAY_PATH="$HOME/ros_ws/underlay" && \
    if [ -f /tmp/consolidated.repos ]; then \
        vcs import --recursive "$ROS_UNDERLAY_PATH" < /tmp/consolidated.repos || true; \
    fi

# Build the underlay workspace as user with proper environment variables
RUN export ROS_WORKSPACE_ROOT="$HOME/ros_ws" && \
    export ROS_UNDERLAY_PATH="$HOME/ros_ws/underlay" && \
    export ROS_UNDERLAY_BUILD="$HOME/ros_ws/underlay_build" && \
    export ROS_UNDERLAY_INSTALL="$HOME/ros_ws/underlay_install" && \
    underlay_deps.sh && underlay_build.sh

# Install rosdeps for the main workspace at container startup when workspace is mounted
RUN echo 'if [ ! -f "$HOME/.rosdeps_installed" ] && [ -d "$HOME/demos" ]; then' >> ~/.bashrc && \
    echo '  install_rosdeps.sh' >> ~/.bashrc && \
    echo 'fi' >> ~/.bashrc


# Update COLCON_DEFAULTS_FILE to use home directory
#need to work out why I can't just copy directly to the right location...
COPY colcon-defaults.yaml /colcon-defaults.yaml
RUN export COLCON_DEFAULTS_FILE="$HOME/ros_ws/colcon-defaults.yaml" && \
    cp /colcon-defaults.yaml "$COLCON_DEFAULTS_FILE" && \
    echo "export COLCON_DEFAULTS_FILE=$COLCON_DEFAULTS_FILE" >> ~/.bashrc

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN printf '%s\n' 'if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then source "${ROS_UNDERLAY_INSTALL}/setup.bash"; fi' >> $HOME/.bashrc

RUN printf '%s\n' '' '# Run colcon build on first container start' 'if [ ! -f "$HOME/.colcon_built" ]; then' '  echo "Running colcon build for first time setup..."' '  source /opt/ros/jazzy/setup.bash' '  if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then' '    source "${ROS_UNDERLAY_INSTALL}/setup.bash"' '  fi' '  colcon build' '  touch $HOME/.colcon_built' 'fi' >> $HOME/.bashrc

RUN printf '%s\n' 'if [ -f "/usr/local/share/vcstool-completion/vcs.bash" ]; then source "/usr/local/share/vcstool-completion/vcs.bash"; fi' >> $HOME/.bashrc

RUN printf '%s\n' 'if [ -f "$ROS_INSTALL_BASE/setup.bash" ]; then source "$ROS_INSTALL_BASE/setup.bash"; fi' >> $HOME/.bashrc

# Set WORKDIR to the expanded path using a RUN command
RUN export ROS_WORKSPACE_ROOT="$HOME/ros_ws" && echo "WORKDIR will be: $ROS_WORKSPACE_ROOT"
WORKDIR /home/ags/ros_ws
