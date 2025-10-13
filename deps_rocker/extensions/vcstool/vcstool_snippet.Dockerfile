RUN --mount=type=cache,target=/root/.cache/pip,id=pip-cache pip install vcstool --break-system-packages

# Define the canonical ROS workspace layout
ENV ROS_WORKSPACE_ROOT=/ros_ws
ENV ROS_UNDERLAY_PATH=/ros_ws/underlay
ENV ROS_UNDERLAY_BUILD=/ros_ws/underlay_build
ENV ROS_UNDERLAY_INSTALL=/ros_ws/underlay_install
ENV ROS_BUILD_BASE=/ros_ws/build
ENV ROS_INSTALL_BASE=/ros_ws/install
ENV ROS_LOG_BASE=/ros_ws/log

RUN mkdir -p "$ROS_UNDERLAY_PATH" "$ROS_UNDERLAY_BUILD" "$ROS_UNDERLAY_INSTALL" \
  "$ROS_BUILD_BASE" "$ROS_INSTALL_BASE" "$ROS_LOG_BASE" \
  && chmod -R 777 /ros_ws

# Import the consolidated depends.repos manifest to underlay
COPY consolidated.repos /ros_ws/consolidated.repos
RUN --mount=type=cache,target=/root/.cache/vcs-repos,id=vcs-repos-cache \
    mkdir -p /root/.cache/vcs-repos/underlay && \
    vcs import --recursive /root/.cache/vcs-repos/underlay < /ros_ws/consolidated.repos && \
    cp -r /root/.cache/vcs-repos/underlay/. /ros_ws/underlay/
