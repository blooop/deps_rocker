# Copy build-underlay script to /usr/local/bin (always install, regardless of repos)
COPY @(extension_name)/build-underlay.sh /usr/local/bin/build-underlay
RUN chmod +x /usr/local/bin/build-underlay

ENV ROS_WORKSPACE_ROOT=@(workspace_root)
ENV ROS_REPOS_ROOT=@(repos_root)
ENV ROS_DEPENDENCIES_ROOT=@(dependencies_root)
ENV ROS_UNDERLAY_PATH=@(underlay_path)
ENV ROS_BUILD_BASE=@(workspace_root)/build
ENV ROS_LOG_BASE=@(workspace_root)/log
ENV COLCON_LOG_PATH=@(workspace_root)/log

RUN mkdir -p @(repos_root) @(dependencies_root) @(underlay_path) @(workspace_root)/build @(workspace_root)/log

# Build ROS underlay using the shared script to keep behavior consistent everywhere
RUN --mount=type=cache,target=/root/.ros/rosdep,sharing=locked,id=rosdep-cache \
    --mount=type=cache,target=@(workspace_root)/build,sharing=locked,id=colcon-build-cache \
    --mount=type=cache,target=/root/.colcon,sharing=locked,id=colcon-cache \
    build-underlay

# Set environment variables to include underlay (even if empty)
ENV AMENT_PREFIX_PATH=@(underlay_path):${AMENT_PREFIX_PATH}
ENV COLCON_PREFIX_PATH=@(underlay_path):${COLCON_PREFIX_PATH}
ENV LD_LIBRARY_PATH=@(underlay_path)/lib:${LD_LIBRARY_PATH}
ENV PATH=@(underlay_path)/bin:${PATH}
ENV PYTHONPATH=@(underlay_path)/lib/python3.12/site-packages:${PYTHONPATH}

RUN chmod -R a+rwX @(underlay_path) @(workspace_root)/build @(workspace_root)/log
