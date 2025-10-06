# Copy build-underlay script to /usr/local/bin (always install, regardless of repos)
COPY @(extension_name)/build-underlay.sh /usr/local/bin/build-underlay
RUN chmod +x /usr/local/bin/build-underlay

ENV ROS_WORKSPACE_ROOT=@(workspace_root)
ENV ROS_REPOS_ROOT=@(repos_root)
ENV ROS_DEPENDENCIES_ROOT=@(dependencies_root)
ENV ROS_UNDERLAY_PATH=@(underlay_path)

RUN mkdir -p @(repos_root) @(dependencies_root) @(underlay_path) @(workspace_root)/build @(workspace_root)/log

@[if depend_repos]@
# Build ROS underlay from vcstool repositories
RUN --mount=type=cache,target=/root/.ros/rosdep,sharing=locked,id=rosdep-cache \
    rosdep update || true

# Collect all repository paths for building
@[for dep in depend_repos]@
RUN echo "@(dependencies_root)/@(dep['path'])" >> /tmp/ros_underlay_paths.txt
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

# Build packages with colcon to the underlay path
RUN --mount=type=cache,target=@(workspace_root)/build,sharing=locked,id=colcon-build-cache \
    --mount=type=cache,target=/root/.colcon,sharing=locked,id=colcon-cache \
    if [ -f /tmp/ros_underlay_paths.txt ]; then \
        . /opt/ros/jazzy/setup.sh && \
        mkdir -p @(underlay_path) && \
        while IFS= read -r path; do \
            if [ -d "$path" ] && [ "$(find "$path" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)" -gt 0 ]; then \
                cd "$path" && \
                colcon build --install-base @(underlay_path) --merge-install; \
            fi; \
        done < /tmp/ros_underlay_paths.txt && \
        rm /tmp/ros_underlay_paths.txt; \
    else \
        mkdir -p @(underlay_path); \
    fi
@[else]@
# No *.repos files found, create empty underlay directory
RUN mkdir -p @(underlay_path) @(workspace_root)/build @(workspace_root)/log
@[end if]@

# Set environment variables to include underlay (even if empty)
ENV AMENT_PREFIX_PATH=@(underlay_path):$AMENT_PREFIX_PATH
ENV COLCON_PREFIX_PATH=@(underlay_path):$COLCON_PREFIX_PATH
ENV LD_LIBRARY_PATH=@(underlay_path)/lib:$LD_LIBRARY_PATH
ENV PATH=@(underlay_path)/bin:$PATH
ENV PYTHONPATH=@(underlay_path)/lib/python3.12/site-packages:$PYTHONPATH

RUN chmod -R a+rwX @(underlay_path) @(workspace_root)/build @(workspace_root)/log
