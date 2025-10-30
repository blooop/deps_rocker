# Change ownership of ROS workspace directories to the user
# This runs after the user is created, ensuring they can write to build directories
RUN chown -R ${USER_NAME}:${USER_NAME} /ros_ws
  
#ROS user snippet
RUN DEPS_ROOT="${ROS_DEPENDENCIES_ROOT}" && \
    if [ -d "$DEPS_ROOT" ]; then \
        rosdep update && \
        rosdep install --from-paths "$DEPS_ROOT" --ignore-src -r -y; \
    fi


#need to work out why I can't just copy directly to the right location...
COPY colcon-defaults.yaml /colcon-defaults.yaml
RUN cp /colcon-defaults.yaml $COLCON_DEFAULTS_FILE

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN printf '%s\n' 'if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then source "${ROS_UNDERLAY_INSTALL}/setup.bash"; fi' >> $HOME/.bashrc

RUN printf '%s\n' '' '# Run colcon build on first container start' 'if [ ! -f "$HOME/.colcon_built" ]; then' '  echo "Running colcon build for first time setup..."' '  source /opt/ros/jazzy/setup.bash' '  # Check if current workspace has packages and install their dependencies' '  if [ -n "$(find "${ROS_WORKSPACE_ROOT}" -name "package.xml" -print -quit)" ]; then' '    echo "Installing rosdep dependencies for current workspace..."' '    rosdep update || true' '    rosdep install --from-paths "${ROS_WORKSPACE_ROOT}" --ignore-src -r -y || true' '  fi' '  # Source underlay if it was built during Docker build stage' '  if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then' '    source "${ROS_UNDERLAY_INSTALL}/setup.bash"' '  fi' '  colcon build' '  touch $HOME/.colcon_built' 'fi' >> $HOME/.bashrc

RUN printf '%s\n' 'if [ -f "/usr/local/share/vcstool-completion/vcs.bash" ]; then source "/usr/local/share/vcstool-completion/vcs.bash"; fi' >> $HOME/.bashrc

RUN printf '%s\n' 'if [ -f "$ROS_INSTALL_BASE/setup.bash" ]; then source "$ROS_INSTALL_BASE/setup.bash"; fi' >> $HOME/.bashrc

WORKDIR $ROS_WORKSPACE_ROOT
