# Import the consolidated depends.repos manifest to underlay with user cache
RUN --mount=type=cache,target=$HOME/.cache/vcs-repos,id=vcs-repos-cache \
    rm -rf $HOME/.cache/vcs-repos/underlay && \
    mkdir -p $HOME/.cache/vcs-repos/underlay && \
    vcs import --recursive $HOME/.cache/vcs-repos/underlay < /ros_ws/consolidated.repos && \
    cp -r $HOME/.cache/vcs-repos/underlay/. /ros_ws/underlay/

# Install dependencies and build the underlay workspace
RUN --mount=type=cache,target=$HOME/.ros/rosdep,id=rosdep-cache \
    underlay_deps.sh && underlay_build.sh
  
# Install additional dependencies if ROS_DEPENDENCIES_ROOT is defined
RUN if [ -n "${ROS_DEPENDENCIES_ROOT:-}" ] && [ -d "${ROS_DEPENDENCIES_ROOT}" ]; then \
        rosdep update && \
        rosdep install --from-paths "${ROS_DEPENDENCIES_ROOT}" --ignore-src -r -y; \
    fi


#need to work out why I can't just copy directly to the right location...
COPY colcon-defaults.yaml /colcon-defaults.yaml
RUN cp /colcon-defaults.yaml $COLCON_DEFAULTS_FILE

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN printf '%s\n' 'if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then source "${ROS_UNDERLAY_INSTALL}/setup.bash"; fi' >> $HOME/.bashrc

RUN printf '%s\n' '' '# Run colcon build on first container start' 'if [ ! -f "$HOME/.colcon_built" ]; then' '  echo "Running colcon build for first time setup..."' '  source /opt/ros/jazzy/setup.bash' '  if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then' '    source "${ROS_UNDERLAY_INSTALL}/setup.bash"' '  fi' '  colcon build' '  touch $HOME/.colcon_built' 'fi' >> $HOME/.bashrc

RUN printf '%s\n' 'if [ -f "/usr/local/share/vcstool-completion/vcs.bash" ]; then source "/usr/local/share/vcstool-completion/vcs.bash"; fi' >> $HOME/.bashrc

RUN printf '%s\n' 'if [ -f "$ROS_INSTALL_BASE/setup.bash" ]; then source "$ROS_INSTALL_BASE/setup.bash"; fi' >> $HOME/.bashrc

WORKDIR $ROS_WORKSPACE_ROOT
