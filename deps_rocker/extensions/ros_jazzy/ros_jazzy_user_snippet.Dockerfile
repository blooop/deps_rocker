
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
RUN printf '%s\n' '[ -n "${ROS_UNDERLAY_INSTALL:-}" ] && [ -f "${ROS_UNDERLAY_INSTALL}/setup.bash" ] && source "${ROS_UNDERLAY_INSTALL}/setup.bash"' >> $HOME/.bashrc

RUN echo "\n# Run colcon build on first container start\nif [ ! -f \"\$HOME/.colcon_built\" ]; then\n  echo 'Running colcon build for first time setup...'\n  colcon build\n  touch \$HOME/.colcon_built\nfi\n" >> $HOME/.bashrc

RUN echo 'source $ROS_INSTALL_BASE/setup.bash' >> $HOME/.bashrc

WORKDIR $ROS_WORKSPACE_ROOT