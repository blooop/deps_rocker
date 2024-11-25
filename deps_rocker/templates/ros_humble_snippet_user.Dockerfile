
#ROS user snippet

# RUN rosdep update

# COPY dependencies.repos dependencies.repos
# RUN vsc import < dependencies.repos

# RUN rosdep install --from-paths src --ignore-src -r -y