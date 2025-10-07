#from https://github.com/athackst/dockerfiles/blob/main/ros2/jazzy.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
  && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
  curl \
  gnupg2 \
  lsb-release \
  sudo \
  software-properties-common \
  wget \
  python3-pip \
  cmake \
  build-essential \
  && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-ros-core \
  python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

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

RUN mkdir -p /workspaces/ros_ws/{repos,src,underlay,build,log} && chmod -R 777 /workspaces/ros_ws

ENV ROS_WORKSPACE_ROOT=/workspaces/ros_ws
ENV ROS_REPOS_ROOT=/workspaces/ros_ws/repos
ENV ROS_DEPENDENCIES_ROOT=/workspaces/ros_ws/src
ENV ROS_UNDERLAY_PATH=/workspaces/ros_ws/underlay
ENV ROS_BUILD_BASE=/workspaces/ros_ws/build
ENV ROS_LOG_BASE=/workspaces/ros_ws/log
ENV COLCON_LOG_PATH=/workspaces/ros_ws/log
