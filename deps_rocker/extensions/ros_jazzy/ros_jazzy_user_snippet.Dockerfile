# Override workspace paths to use home directory
ENV ROS_WORKSPACE_ROOT=$HOME/ros_ws
ENV ROS_UNDERLAY_PATH=$HOME/ros_ws/underlay
ENV ROS_UNDERLAY_BUILD=$HOME/ros_ws/underlay_build
ENV ROS_UNDERLAY_INSTALL=$HOME/ros_ws/underlay_install
ENV ROS_BUILD_BASE=$HOME/ros_ws/build
ENV ROS_INSTALL_BASE=$HOME/ros_ws/install
ENV ROS_LOG_BASE=$HOME/ros_ws/log
ENV COLCON_LOG_PATH=$HOME/ros_ws/log

# Create ROS workspace directories as user
RUN mkdir -p "$HOME/ros_ws/underlay" "$HOME/ros_ws/underlay_build" "$HOME/ros_ws/underlay_install" \
  "$HOME/ros_ws/build" "$HOME/ros_ws/install" "$HOME/ros_ws/log"

# Import the consolidated depends.repos manifest to underlay as user
RUN if [ -f /tmp/consolidated.repos ]; then \
        vcs import --recursive "$ROS_UNDERLAY_PATH" < /tmp/consolidated.repos || true; \
    fi

# Build the underlay workspace as user if it contains packages
RUN underlay_deps.sh && underlay_build.sh

#ROS user snippet
RUN DEPS_ROOT="${ROS_DEPENDENCIES_ROOT}" && \
    if [ -d "$DEPS_ROOT" ]; then \
        rosdep update && \
        rosdep install --from-paths "$DEPS_ROOT" --ignore-src -r -y; \
    fi


# Update COLCON_DEFAULTS_FILE to use home directory
ENV COLCON_DEFAULTS_FILE=$HOME/ros_ws/colcon-defaults.yaml

#need to work out why I can't just copy directly to the right location...
COPY colcon-defaults.yaml /colcon-defaults.yaml
RUN cp /colcon-defaults.yaml $COLCON_DEFAULTS_FILE

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN printf '%s\n' 'if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then source "${ROS_UNDERLAY_INSTALL}/setup.bash"; fi' >> $HOME/.bashrc

RUN printf '%s\n' '' '# Run colcon build on first container start' 'if [ ! -f "$HOME/.colcon_built" ]; then' '  echo "Running colcon build for first time setup..."' '  source /opt/ros/jazzy/setup.bash' '  if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then' '    source "${ROS_UNDERLAY_INSTALL}/setup.bash"' '  fi' '  colcon build' '  touch $HOME/.colcon_built' 'fi' >> $HOME/.bashrc

RUN printf '%s\n' 'if [ -f "/usr/local/share/vcstool-completion/vcs.bash" ]; then source "/usr/local/share/vcstool-completion/vcs.bash"; fi' >> $HOME/.bashrc

RUN printf '%s\n' 'if [ -f "$ROS_INSTALL_BASE/setup.bash" ]; then source "$ROS_INSTALL_BASE/setup.bash"; fi' >> $HOME/.bashrc

WORKDIR $ROS_WORKSPACE_ROOT
