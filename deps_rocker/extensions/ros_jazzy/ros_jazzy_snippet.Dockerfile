#from https://github.com/athackst/dockerfiles/blob/main/ros2/jazzy.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive


# Install ROS2 repository and key with apt cache mount
RUN --mount=type=cache,target=/var/cache/apt,id=apt-cache \
  sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-ros-core \
  python3-dev \
  python3-numpy

# Install colcon, vcstool, numpy, and lark via pip
# Installing numpy via pip ensures CMake can find Python3_NumPy_INCLUDE_DIRS
# lark is required by rosidl_parser
RUN --mount=type=cache,target=/root/.cache/pip,id=pip-cache \
  pip install colcon-common-extensions colcon-defaults colcon-spawn-shell colcon-runner colcon-clean rosdep colcon-top-level-workspace vcstool numpy lark --break-system-packages

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV COLCON_LOG_PATH=/tmp/ros_ws_template/ros_ws/log

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
  else \
    echo "rosdep already initialized, skipping init"; \
  fi

# Define the canonical ROS workspace layout in /tmp for root-owned build
# These will be copied to user home directory in user snippet
ENV ROS_WORKSPACE_ROOT=/tmp/ros_ws_template
ENV ROS_UNDERLAY_PATH=/tmp/ros_ws_template/underlay_ws
ENV ROS_UNDERLAY_BUILD=/tmp/ros_ws_template/underlay_ws/build
ENV ROS_UNDERLAY_INSTALL=/tmp/ros_ws_template/underlay_ws/install
ENV ROS_BUILD_BASE=/tmp/ros_ws_template/ros_ws/build
ENV ROS_INSTALL_BASE=/tmp/ros_ws_template/ros_ws/install
ENV ROS_LOG_BASE=/tmp/ros_ws_template/ros_ws/log

RUN mkdir -p "$ROS_UNDERLAY_PATH" "$ROS_UNDERLAY_BUILD" "$ROS_UNDERLAY_INSTALL" \
  "$ROS_BUILD_BASE" "$ROS_INSTALL_BASE" "$ROS_LOG_BASE"

# Import the consolidated depends.repos manifest to underlay
COPY consolidated.repos /tmp/ros_ws_template/consolidated.repos
RUN --mount=type=cache,target=/root/.cache/vcs-repos,id=vcs-repos-cache \
    rm -rf /root/.cache/vcs-repos/underlay && \
    mkdir -p /root/.cache/vcs-repos/underlay && \
    vcs import --recursive /root/.cache/vcs-repos/underlay < /tmp/ros_ws_template/consolidated.repos && \
    cp -r /root/.cache/vcs-repos/underlay/. /tmp/ros_ws_template/underlay_ws/

# Install underlay build scripts
COPY underlay_deps.sh underlay_build.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/underlay_deps.sh /usr/local/bin/underlay_build.sh

# Copy test files for the test script
COPY test_package.xml test_setup.py /tmp/

# Build the underlay workspace if it contains packages
RUN --mount=type=cache,target=/var/cache/apt,id=apt-cache \
    --mount=type=cache,target=/root/.ros/rosdep,id=rosdep-cache \
    underlay_deps.sh && underlay_build.sh

ENV COLCON_DEFAULTS_FILE=/tmp/ros_ws_template/colcon-defaults.yaml
