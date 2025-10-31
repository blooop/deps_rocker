# Generic ROS 2 Jazzy setup - works with any ROS repository
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 repository and key
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-core \
    python3-vcs2l \
    python3-rosdep

# Install colcon, vcstool, numpy, and lark via pip
RUN --mount=type=cache,target=/root/.cache/pip,id=pip-cache \
    pip install colcon-common-extensions colcon-defaults colcon-runner colcon-clean lark --break-system-packages

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
  else \
    echo "rosdep already initialized, skipping init"; \
  fi


# Copy scripts and make them executable
COPY underlay_deps.sh underlay_build.sh install_rosdeps.sh /usr/local/bin/
COPY rosdep_underlay.sh rosdep_overlay.sh build_underlay.sh update_repos.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/underlay_deps.sh /usr/local/bin/underlay_build.sh /usr/local/bin/install_rosdeps.sh \
             /usr/local/bin/rosdep_underlay.sh /usr/local/bin/rosdep_overlay.sh \
             /usr/local/bin/build_underlay.sh /usr/local/bin/update_repos.sh

