RUN --mount=type=cache,target=/root/.cache/pip,id=pip-cache pip install vcstool --break-system-packages

# Define the canonical ROS workspace layout
ENV ROS_WORKSPACE_ROOT=/ros_ws
ENV ROS_REPOS_ROOT=/ros_ws/repos
ENV ROS_DEPENDENCIES_ROOT=/ros_ws/src
ENV ROS_DEPENDS_ROOT=/ros_ws/depends
ENV ROS_UNDERLAY_PATH=/ros_ws/underlay
ENV ROS_BUILD_BASE=/ros_ws/build
ENV ROS_INSTALL_BASE=/ros_ws/install
ENV ROS_LOG_BASE=/ros_ws/log

RUN mkdir -p "$ROS_REPOS_ROOT" "$ROS_DEPENDENCIES_ROOT" "$ROS_DEPENDS_ROOT" "$ROS_UNDERLAY_PATH" \
  "$ROS_BUILD_BASE" "$ROS_INSTALL_BASE" "$ROS_LOG_BASE" \
  && chmod -R 777 /ros_ws

# Import the consolidated depends.repos manifest
COPY consolidated.repos /ros_ws/consolidated.repos
RUN --mount=type=cache,target=/root/.cache/vcs-repos,id=vcs-repos-cache \
    mkdir -p /root/.cache/vcs-repos/depends && \
    vcs import --recursive /root/.cache/vcs-repos/depends < /ros_ws/consolidated.repos && \
    cp -r /root/.cache/vcs-repos/depends/. /ros_ws/depends/
