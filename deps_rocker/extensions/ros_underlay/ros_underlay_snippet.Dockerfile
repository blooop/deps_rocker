@[if depend_repos]@
# Build ROS underlay from vcstool repositories
RUN --mount=type=cache,target=/root/.ros/rosdep,sharing=locked,id=rosdep-cache \
    rosdep update || true

# Collect all repository paths for building
@[for dep in depend_repos]@
RUN echo "/dependencies/@(dep['path'])" >> /tmp/ros_underlay_paths.txt
@[end for]@

# Install dependencies from discovered repo paths with rosdep
RUN --mount=type=cache,target=/root/.ros/rosdep,sharing=locked,id=rosdep-cache \
    if [ -f /tmp/ros_underlay_paths.txt ]; then \
        while IFS= read -r path; do \
            if [ -d "$path" ] && [ "$(find "$path" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -gt 0 ]; then \
                rosdep install --from-paths "$path" --ignore-src -y || true; \
            fi; \
        done < /tmp/ros_underlay_paths.txt; \
    fi

# Build packages with colcon to /opt/ros_underlay
RUN --mount=type=cache,target=/dependencies/build,sharing=locked,id=colcon-build-cache \
    --mount=type=cache,target=/root/.colcon,sharing=locked,id=colcon-cache \
    if [ -f /tmp/ros_underlay_paths.txt ]; then \
        . /opt/ros/jazzy/setup.sh && \
        mkdir -p /opt/ros_underlay && \
        while IFS= read -r path; do \
            if [ -d "$path" ] && [ "$(find "$path" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -gt 0 ]; then \
                cd "$path" && \
                colcon build --install-base /opt/ros_underlay --merge-install; \
            fi; \
        done < /tmp/ros_underlay_paths.txt && \
        rm /tmp/ros_underlay_paths.txt; \
    else \
        mkdir -p /opt/ros_underlay; \
    fi

# Set environment variables to include underlay
ENV AMENT_PREFIX_PATH=/opt/ros_underlay:$AMENT_PREFIX_PATH
ENV COLCON_PREFIX_PATH=/opt/ros_underlay:$COLCON_PREFIX_PATH
ENV LD_LIBRARY_PATH=/opt/ros_underlay/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros_underlay/bin:$PATH
ENV PYTHONPATH=/opt/ros_underlay/lib/python3.12/site-packages:$PYTHONPATH
@[else]@
# No *.repos files found, create empty underlay directory
RUN mkdir -p /opt/ros_underlay
@[end if]@

# Copy build-underlay script to /usr/local/bin
COPY @(extension_name)/build-underlay.sh /usr/local/bin/build-underlay
RUN chmod +x /usr/local/bin/build-underlay
