# Generic ROS 2 Jazzy setup - works with any ROS repository
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 repository and key
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-core \
    python3-vcstool \
    python3-rosdep \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install colcon, vcstool, numpy, and lark via pip
RUN pip install colcon-common-extensions colcon-defaults colcon-spawn-shell colcon-runner colcon-clean rosdep vcstool numpy lark --break-system-packages

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Workspace environment variables for testing
ENV ROS_UNDERLAY_PATH=/opt/ros/underlay/src
ENV ROS_UNDERLAY_BUILD=/opt/ros/underlay/build
ENV ROS_UNDERLAY_INSTALL=/opt/ros/underlay/install

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
  else \
    echo "rosdep already initialized, skipping init"; \
  fi

# Create underlay workspace with proper ownership
RUN mkdir -p /opt/ros/underlay/src /opt/ros/underlay/build /opt/ros/underlay/install && \
    chown -R ${USERNAME}:${USERNAME} /opt/ros/underlay

# Copy scripts and make them executable
COPY underlay_deps.sh underlay_build.sh install_rosdeps.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/underlay_deps.sh /usr/local/bin/underlay_build.sh /usr/local/bin/install_rosdeps.sh

# Copy consolidated repos file if it exists
COPY consolidated.repos /tmp/consolidated.repos
