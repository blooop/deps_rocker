# Copy ROS workspace template from /tmp to user home directory and set permissions
RUN mkdir -p @(user_home_dir)/ros_ws @(user_home_dir)/underlay_ws && \
    cp -r /tmp/ros_ws_template/underlay_ws/. @(user_home_dir)/underlay_ws/ && \
    cp -r /tmp/ros_ws_template/ros_ws/. @(user_home_dir)/ros_ws/ && \
    cp /tmp/ros_ws_template/consolidated.repos @(user_home_dir)/underlay_ws/ && \
    chmod -R 777 @(user_home_dir)/ros_ws @(user_home_dir)/underlay_ws

# Set environment variables to point to user home directories
ENV ROS_WORKSPACE_ROOT=@(user_home_dir)/ros_ws \
    ROS_UNDERLAY_PATH=@(user_home_dir)/underlay_ws \
    ROS_UNDERLAY_BUILD=@(user_home_dir)/underlay_ws/build \
    ROS_UNDERLAY_INSTALL=@(user_home_dir)/underlay_ws/install \
    ROS_BUILD_BASE=@(user_home_dir)/ros_ws/build \
    ROS_INSTALL_BASE=@(user_home_dir)/ros_ws/install \
    ROS_LOG_BASE=@(user_home_dir)/ros_ws/log \
    COLCON_LOG_PATH=@(user_home_dir)/ros_ws/log

# Also export in bashrc for interactive shells
RUN echo "export ROS_WORKSPACE_ROOT=@(user_home_dir)/ros_ws" >> @(user_home_dir)/.bashrc && \
    echo "export ROS_UNDERLAY_PATH=@(user_home_dir)/underlay_ws" >> @(user_home_dir)/.bashrc && \
    echo "export ROS_UNDERLAY_BUILD=@(user_home_dir)/underlay_ws/build" >> @(user_home_dir)/.bashrc && \
    echo "export ROS_UNDERLAY_INSTALL=@(user_home_dir)/underlay_ws/install" >> @(user_home_dir)/.bashrc && \
    echo "export ROS_BUILD_BASE=@(user_home_dir)/ros_ws/build" >> @(user_home_dir)/.bashrc && \
    echo "export ROS_INSTALL_BASE=@(user_home_dir)/ros_ws/install" >> @(user_home_dir)/.bashrc && \
    echo "export ROS_LOG_BASE=@(user_home_dir)/ros_ws/log" >> @(user_home_dir)/.bashrc && \
    echo "export COLCON_LOG_PATH=@(user_home_dir)/ros_ws/log" >> @(user_home_dir)/.bashrc

#ROS user snippet
RUN DEPS_ROOT="${ROS_DEPENDENCIES_ROOT}" && \
    if [ -d "$DEPS_ROOT" ]; then \
        rosdep update && \
        rosdep install --from-paths "$DEPS_ROOT" --ignore-src -r -y; \
    fi

# Copy colcon defaults to user home
COPY colcon-defaults.yaml @(user_home_dir)/colcon-defaults.yaml
ENV COLCON_DEFAULTS_FILE=@(user_home_dir)/colcon-defaults.yaml

RUN echo "source /opt/ros/jazzy/setup.bash" >> @(user_home_dir)/.bashrc
RUN printf '%s\n' 'if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then source "${ROS_UNDERLAY_INSTALL}/setup.bash"; fi' >> @(user_home_dir)/.bashrc

RUN printf '%s\n' '' '# Run colcon build on first container start' 'if [ ! -f "@(user_home_dir)/.colcon_built" ]; then' '  echo "Running colcon build for first time setup..."' '  source /opt/ros/jazzy/setup.bash' '  if [ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ]; then' '    source "${ROS_UNDERLAY_INSTALL}/setup.bash"' '  fi' '  # Try to build in current directory if it has a src folder, otherwise build in ros_ws' '  if [ -d "src" ]; then' '    colcon build' '  else' '    cd $ROS_WORKSPACE_ROOT && colcon build' '  fi' '  touch @(user_home_dir)/.colcon_built' 'fi' >> @(user_home_dir)/.bashrc

RUN printf '%s\n' 'if [ -f "/usr/local/share/vcstool-completion/vcs.bash" ]; then source "/usr/local/share/vcstool-completion/vcs.bash"; fi' >> @(user_home_dir)/.bashrc

RUN printf '%s\n' 'if [ -f "$ROS_INSTALL_BASE/setup.bash" ]; then source "$ROS_INSTALL_BASE/setup.bash"; fi' >> @(user_home_dir)/.bashrc
