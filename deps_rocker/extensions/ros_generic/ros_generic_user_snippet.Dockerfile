# Ensure coreutils (chmod) is available in the final image for test.sh
RUN apt-get update && apt-get install -y --no-install-recommends coreutils && rm -rf /var/lib/apt/lists/*
# This user snippet ensures the ROS environment is always sourced in interactive shells
# and colcon is available in the PATH for the user.

# Add ROS setup sourcing to bashrc for interactive shells
RUN echo '' >> ~/.bashrc \
	&& echo '# Source ROS setup for interactive shells' >> ~/.bashrc \
	&& echo 'if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then' >> ~/.bashrc \
	&& echo '    source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc \
	&& echo 'fi' >> ~/.bashrc
