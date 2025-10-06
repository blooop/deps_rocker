# This user snippet ensures the ROS environment is always sourced in interactive shells
# and colcon is available in the PATH for the user.

# Add ROS setup sourcing to bashrc for interactive shells
RUN echo '\n# Source ROS setup for interactive shells\nif [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then\n    source /opt/ros/$ROS_DISTRO/setup.bash\nfi' >> ~/.bashrc
