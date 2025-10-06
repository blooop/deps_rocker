# Ensure coreutils (chmod) is available in the final image for test.sh
RUN apt-get update && apt-get install -y --no-install-recommends coreutils && rm -rf /var/lib/apt/lists/*

# Ensure coreutils (chmod) is available in the final image for test.sh
RUN apt-get update && apt-get install -y --no-install-recommends coreutils && rm -rf /var/lib/apt/lists/*




# This snippet is appended to the base image provided by the extension system

ARG ROS_DISTRO=@(f"{ros_distro}")

# Copy ROS 2 installation and libraries from the builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/ros /opt/ros/$$${{ROS_DISTRO}}")
@(f"COPY --from={builder_stage} {builder_output_dir}/lib/* /usr/local/lib/")

# Set up environment variables for ROS
ENV ROS_DISTRO=$$${ROS_DISTRO}
ENV AMENT_PREFIX_PATH=/opt/ros/$$${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/$$${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/$$${ROS_DISTRO}/lib:/usr/local/lib
ENV PATH=/opt/ros/$$${ROS_DISTRO}/bin:$$PATH
ENV PYTHONPATH=/opt/ros/$$${ROS_DISTRO}/lib/python3.10/site-packages:/opt/ros/$$${ROS_DISTRO}/local/lib/python3.10/dist-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Underlay management tooling and bootstrap

COPY --from=ros_generic_builder /opt/deps_rocker/ros_generic/ros_underlays.py /usr/local/bin/ros-underlays
COPY --from=ros_generic_builder /opt/deps_rocker/ros_generic/ros_underlays_manifest.json /opt/ros/underlays/ros_underlays_manifest.json


