
#ROS user snippet

# COPY dependencies.repos dependencies.repos
# RUN vsc import < dependencies.repos




RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src -r -y

# RUN rosdep update
# vcs import /opt/ros/kinisi/src < dependencies.repos --recursive
# rosdep install --ignore-src --from-paths . -y &&

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF &&
# source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc
RUN echo "source /usr/share/vcstool-completion/vcs.bash" >> $HOME/.bashrc
# RUN echo "alias colcon='colcon --defaults-file ~/.colcon/defaults.yaml'" >> $HOME/.bashrc

