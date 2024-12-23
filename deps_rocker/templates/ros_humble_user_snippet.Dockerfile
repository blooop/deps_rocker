
#ROS user snippet

RUN if [ -d "/dependencies" ]; then \
    rosdep update && \
    rosdep install --from-paths /dependencies --ignore-src -r -y; \
fi

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF &&
# source install/setup.bash

RUN mkdir $HOME/.colcon

#need to work out why I can't just copy directly to the right location...
COPY defaults.yaml /defaults.yaml
RUN cp /defaults.yaml $HOME/.colcon/defaults.yaml

RUN echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc;
RUN echo "source /workspaces/ros_ws/install/setup.bash" >> $HOME/.bashrc
