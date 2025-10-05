# Spec for generic ROS extension (ros_generic)

- Accepts a ROS 2 distribution argument (default: jazzy)
- Installs and configures the specified ROS 2 version in Docker
- Parameterizes Dockerfile snippet with ARG/ENV for ROS_DISTRO
- Exposes environment variable ROS_DISTRO in container
- Test script validates ros2 and correct ROS_DISTRO
- Entry point: ros_generic = "deps_rocker.extensions.ros_generic.ros_generic:RosGeneric"
- Added to EXTENSIONS_TO_TEST for CI
- Backward compatible: can be used for humble, jazzy, etc.
