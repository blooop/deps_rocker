# syntax=docker/dockerfile:1.4

# Use rwthika/ros2 image as builder stage to get ROS 2 installation
@(f"FROM rwthika/ros2:{ros_distro}-ros-base AS {builder_stage}")


# Install additional development tools and Python packages
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    ccache \
    coreutils \
    vim \
    htop \
    ripgrep \
    fd-find \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-defaults \
    python3-colcon-clean \
    python3-lark \
    python3-numpy \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Configure ccache for standard usage
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_DIR=/root/.ccache
RUN ccache --set-config=max_size=20G && \
    ccache --set-config=compression=true && \
    ccache --set-config=compression_level=6


# Install Python tools not available in apt
RUN pip install --break-system-packages colcon-spawn-shell colcon-ros-bundle colcon-runner uv

# Initialize rosdep
RUN rosdep init || true


# Create ros-underlays-rebuild script (escape $ and @ for empy)
RUN echo '#!/usr/bin/env bash\nset -euo pipefail\nexec ros-underlays rebuild "$$@@"' > /usr/local/bin/ros-underlays-rebuild && chmod +x /usr/local/bin/ros-underlays-rebuild


COPY ros_underlays.py @(f"{builder_output_dir}")/ros_underlays.py
COPY ros_underlays_manifest.json @(f"{builder_output_dir}")/ros_underlays_manifest.json
RUN mkdir -p @(f"{builder_output_dir}") && cp -r /opt/ros @(f"{builder_output_dir}")/ && mkdir -p @(f"{builder_output_dir}")/lib && find /usr/local/lib -name 'lib*.so*' -exec cp -P {} @(f"{builder_output_dir}")/lib/ \; 2>/dev/null || true && find /usr/lib -name 'libspdlog*' -exec cp -P {} @(f"{builder_output_dir}")/lib/ \; 2>/dev/null || true && find /usr/lib -name 'libfmt*' -exec cp -P {} @(f"{builder_output_dir}")/lib/ \; 2>/dev/null || true && ldconfig -p | grep -E '(spdlog|fmt)' | awk '{print $$NF}' | xargs -I {} cp -P {} @(f"{builder_output_dir}")/lib/ 2>/dev/null || true
