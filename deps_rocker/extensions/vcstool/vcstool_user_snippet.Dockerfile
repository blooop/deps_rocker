# Change ownership of ROS workspace directories to the user
# This runs after the user is created, ensuring they can write to build directories
RUN if [ -d "/ros_ws" ]; then \
    chown -R ${USER_NAME}:${USER_NAME} /ros_ws; \
  fi
