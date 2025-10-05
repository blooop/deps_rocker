



# This snippet is appended to the base image provided by the extension system
ARG ROS_DISTRO=@ros_distro@

# Copy ROS 2 installation and libraries from the builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/ros /opt/ros")
@(f"COPY --from={builder_stage} {builder_output_dir}/lib/* /usr/local/lib/")

# Set up environment variables for ROS
ENV ROS_DISTRO=${ROS_DISTRO}
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:/usr/local/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.10/site-packages:/opt/ros/${ROS_DISTRO}/local/lib/python3.10/dist-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Install additional development tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ccache \
    vim \
    htop \
    ripgrep \
    fd-find \
    python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Configure ccache for standard usage
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_DIR=/root/.ccache
RUN ccache --set-config=max_size=20G && \
    ccache --set-config=compression=true && \
    ccache --set-config=compression_level=6

# Install colcon and other Python tools
RUN pip install colcon-common-extensions colcon-defaults colcon-spawn-shell colcon-clean rosdep

# Initialize rosdep
RUN rosdep init || true

# Configure ccache for standard usage
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_DIR=/root/.ccache
RUN ccache --set-config=max_size=20G && \
    ccache --set-config=compression=true && \
    ccache --set-config=compression_level=6

# Install colcon and rosdep Python tools
RUN pip install colcon-common-extensions colcon-defaults colcon-spawn-shell colcon-clean colcon-runner uv rosdep lark numpy

# Initialize rosdep
RUN rosdep init || true
