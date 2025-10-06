# Build ROS underlay from vcstool repositories
RUN --mount=type=cache,target=/root/.ros/rosdep,sharing=locked,id=rosdep-cache \
    rosdep update || true

# Install dependencies from /dependencies with rosdep (if directory exists)
RUN --mount=type=cache,target=/root/.ros/rosdep,sharing=locked,id=rosdep-cache \
    if [ -d "/dependencies" ] && [ "$(find /dependencies -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -gt 0 ]; then \
        rosdep install --from-paths /dependencies --ignore-src -y || true; \
    fi

# Build packages with colcon to /opt/ros_underlay (if packages exist)
RUN --mount=type=cache,target=/dependencies/build,sharing=locked,id=colcon-build-cache \
    --mount=type=cache,target=/root/.colcon,sharing=locked,id=colcon-cache \
    if [ -d "/dependencies" ] && [ "$(find /dependencies -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -gt 0 ]; then \
        cd /dependencies && \
        . /opt/ros/jazzy/setup.sh && \
        colcon build --install-base /opt/ros_underlay --merge-install; \
    else \
        mkdir -p /opt/ros_underlay && \
        echo "No packages found in /dependencies, skipping underlay build"; \
    fi

# Set environment variables to include underlay
ENV AMENT_PREFIX_PATH=/opt/ros_underlay:$AMENT_PREFIX_PATH
ENV COLCON_PREFIX_PATH=/opt/ros_underlay:$COLCON_PREFIX_PATH
ENV LD_LIBRARY_PATH=/opt/ros_underlay/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros_underlay/bin:$PATH
ENV PYTHONPATH=/opt/ros_underlay/lib/python3.12/site-packages:$PYTHONPATH

# Copy build-underlay script to /usr/local/bin
COPY @(extension_name)/build-underlay.sh /usr/local/bin/build-underlay
RUN chmod +x /usr/local/bin/build-underlay
