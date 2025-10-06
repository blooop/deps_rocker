from deps_rocker.extensions.ros_generic.ros_generic import RosGeneric


class RosJazzy(RosGeneric):
    """Install ROS Jazzy for development (legacy entrypoint, uses generic class)"""

    name = "ros_jazzy"
    default_ros_distro = "jazzy"
