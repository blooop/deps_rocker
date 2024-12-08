
#ROS user snippet
RUN  rosdep update; rosdep install --from-paths /dependencies --ignore-src -r -y

# RUN rosdep update
# vcs import /opt/ros/kinisi/src < dependencies.repos --recursive
# rosdep install --ignore-src --from-paths /dependencies -y -r

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF &&
# source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc
RUN echo "source /usr/share/vcstool-completion/vcs.bash" >> $HOME/.bashrc
# RUN echo "alias colcon='colcon --defaults-file ~/.colcon/defaults.yaml'" >> $HOME/.bashrc

