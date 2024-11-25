
#ROS user snippet

# COPY dependencies.repos dependencies.repos
# RUN vsc import < dependencies.repos

# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN rosdep update

RUN echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc