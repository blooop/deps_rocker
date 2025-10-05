



# This snippet is appended to the base image provided by the extension system.
ARG ROS_DISTRO=humble

# Set up environment variables for ROS
ENV ROS_DISTRO=${ROS_DISTRO}
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Install additional tools needed for development and testing
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    git \
    git-lfs \
    build-essential \
    cmake \
    ccache \
    curl \
    wget \
    vim \
    htop \
    ripgrep \
    fd-find \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Configure ccache for standard usage
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_DIR=/root/.ccache
RUN ccache --set-config=max_size=20G && \
    ccache --set-config=compression=true && \
    ccache --set-config=compression_level=6

# Install colcon and rosdep Python tools
RUN pip install colcon-common-extensions colcon-defaults colcon-spawn-shell colcon-clean uv rosdep

# Initialize rosdep
RUN rosdep init || true
