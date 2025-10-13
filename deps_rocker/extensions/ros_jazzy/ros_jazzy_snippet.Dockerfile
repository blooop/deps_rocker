#from https://github.com/athackst/dockerfiles/blob/main/ros2/jazzy.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive


# Install ROS2 repository and key with apt cache mount
RUN --mount=type=cache,target=/var/cache/apt,id=apt-cache \
  sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-ros-core 

RUN --mount=type=cache,target=/root/.cache/pip,id=pip-cache pip install colcon-common-extensions colcon-defaults colcon-spawn-shell colcon-runner colcon-clean rosdep colcon-top-level-workspace --break-system-packages

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
  else \
    echo "rosdep already initialized, skipping init"; \
  fi

RUN mkdir -p /ros_ws/{repos,src,underlay,build,log} && chmod -R 777 /ros_ws

ENV ROS_WORKSPACE_ROOT=/ros_ws
ENV ROS_REPOS_ROOT=/ros_ws/repos
ENV ROS_DEPENDENCIES_ROOT=/ros_ws/src
ENV ROS_UNDERLAY_PATH=/ros_ws/underlay
ENV ROS_BUILD_BASE=/ros_ws/build
ENV ROS_INSTALL_BASE=/ros_ws/install
ENV ROS_LOG_BASE=/ros_ws/log
ENV COLCON_LOG_PATH=/ros_ws/log
ENV COLCON_LOG_PATH=/ros_ws/log
ENV COLCON_DEFAULTS_FILE=/ros_ws/colcon-defaults.yaml

RUN mkdir -p "$ROS_REPOS_ROOT" "$ROS_DEPENDENCIES_ROOT" "$ROS_UNDERLAY_PATH" "$ROS_BUILD_BASE" "$ROS_LOG_BASE" \
  "$ROS_INSTALL_BASE" \
  && chmod -R 777 /ros_ws 
