



# This snippet is appended to the base image provided by the extension system
ARG ROS_DISTRO=@ros_distro@

# Copy ROS 2 installation and libraries from the builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/ros /opt/ros/$${{ROS_DISTRO}}")
@(f"COPY --from={builder_stage} {builder_output_dir}/lib/* /usr/local/lib/")

# Set up environment variables for ROS
ENV ROS_DISTRO=$${ROS_DISTRO}
ENV AMENT_PREFIX_PATH=/opt/ros/$${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/$${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/$${ROS_DISTRO}/lib:/usr/local/lib
ENV PATH=/opt/ros/$${ROS_DISTRO}/bin:$$PATH
ENV PYTHONPATH=/opt/ros/$${ROS_DISTRO}/lib/python3.10/site-packages:/opt/ros/$${ROS_DISTRO}/local/lib/python3.10/dist-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Underlay management tooling and bootstrap
COPY ros_underlays.py /usr/local/bin/ros-underlays
COPY ros_underlays_manifest.json /opt/ros/underlays/ros_underlays_manifest.json
RUN chmod +x /usr/local/bin/ros-underlays

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


# Install colcon, rosdep, and other Python tools using apt where possible
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-colcon-defaults \
    python3-colcon-clean \
    python3-colcon-runner \
    python3-rosdep \
    python3-lark \
    python3-numpy \
    python3-uv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install colcon-spawn-shell via pip (not available in apt)
RUN pip install --break-system-packages colcon-spawn-shell

# Initialize rosdep
RUN rosdep init || true

RUN ros-underlays sync --verbose && \
    chmod -R a+rwX /opt/ros/underlays || true
RUN echo "if [ -f /opt/ros/underlays/setup.bash ]; then source /opt/ros/underlays/setup.bash; fi" >> /etc/bash.bashrc
RUN printf '#!/usr/bin/env bash\nset -euo pipefail\nexec ros-underlays rebuild "$@"\n' > /usr/local/bin/ros-underlays-rebuild && \
    chmod +x /usr/local/bin/ros-underlays-rebuild
RUN echo "alias ros-underlays-rebuild='ros-underlays rebuild'" >> /etc/bash.bashrc
