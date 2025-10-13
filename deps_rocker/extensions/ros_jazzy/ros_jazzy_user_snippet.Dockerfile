
#ROS user snippet
RUN DEPS_ROOT="${ROS_DEPENDENCIES_ROOT}" && \
    if [ -d "$DEPS_ROOT" ]; then \
        rosdep update && \
        rosdep install --from-paths "$DEPS_ROOT" --ignore-src -r -y; \
    fi


#need to work out why I can't just copy directly to the right location...
COPY defaults.yaml /defaults.yaml
RUN cp /defaults.yaml $COLCON_DEFAULTS_FILE

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN printf '%s\n' '[ -n "${ROS_UNDERLAY_PATH:-}" ] && [ -f "${ROS_UNDERLAY_PATH}/setup.bash" ] && source "${ROS_UNDERLAY_PATH}/setup.bash"' >> $HOME/.bashrc

ENV ROS_FIRST_BUILD=1
RUN echo "\n# Run colcon build on first container start\nif [ \"\$ROS_FIRST_BUILD\" = \"1\" ]; then\n  echo 'Running colcon build for first time setup...'\n  colcon build\n  export ROS_FIRST_BUILD=0\nfi\n" >> $HOME/.bashrc

RUN echo 'source $ROS_INSTALL_BASE/setup.bash' >> $HOME/.bashrc