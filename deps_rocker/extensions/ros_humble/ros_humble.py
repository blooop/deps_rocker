from deps_rocker.extensions.ros_generic.ros_generic import RosGeneric


class RosHumble(RosGeneric):
    """Install ROS Humble for development (legacy entrypoint, uses generic class)"""

    name = "ros_humble"
    default_ros_distro = "humble"
