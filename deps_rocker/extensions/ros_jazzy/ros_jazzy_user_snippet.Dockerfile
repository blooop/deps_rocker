
#ROS user snippet

RUN DEPS_ROOT="${ROS_DEPENDENCIES_ROOT:-/workspaces/ros_ws/src}" && \
    if [ -d "$DEPS_ROOT" ]; then \
        rosdep update && \
        rosdep install --from-paths "$DEPS_ROOT" --ignore-src -r -y; \
    fi

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF &&
# source install/setup.bash

RUN mkdir -p $HOME/.colcon

#need to work out why I can't just copy directly to the right location...
COPY defaults.yaml /defaults.yaml
RUN cp /defaults.yaml $HOME/.colcon/defaults.yaml

RUN echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
RUN printf '%s\n' '[ -n "${ROS_UNDERLAY_PATH:-}" ] && [ -f "${ROS_UNDERLAY_PATH}/setup.bash" ] && source "${ROS_UNDERLAY_PATH}/setup.bash"' >> $HOME/.bashrc
