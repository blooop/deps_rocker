# syntax=docker/dockerfile:1.4

# Use rwthika/ros2 image as builder stage to get ROS 2 installation
@(f"FROM rwthika/ros2:{ros_distro}-ros-core AS {builder_stage}")

# Copy the entire ROS installation to a known location for later copying
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create output directory and copy ROS installation and libraries
@(f"RUN mkdir -p {builder_output_dir} && \\")
    cp -r /opt/ros @(f"{builder_output_dir}/") && \
    mkdir -p @(f"{builder_output_dir}/lib") && \
    find /usr/local/lib -name 'lib*.so*' -exec cp -P {} @(f"{builder_output_dir}/lib/") \; 2>/dev/null || true && \
    find /usr/lib -name 'libspdlog*' -exec cp -P {} @(f"{builder_output_dir}/lib/") \; 2>/dev/null || true && \
    find /usr/lib -name 'libfmt*' -exec cp -P {} @(f"{builder_output_dir}/lib/") \; 2>/dev/null || true && \
    ldconfig -p | grep -E '(spdlog|fmt)' | awk '{print $$NF}' | xargs -I {} cp -P {} @(f"{builder_output_dir}/lib/") 2>/dev/null || true
