from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class UrdfViz(SimpleRockerExtension):
    """Add the urdf-viz to your docker image"""

    name = "urdf_viz"
    depends_on_extensions = ["curl", "ros_humble"]
