from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class UrdfViz(SimpleRockerExtension):
    """Add the urdf-viz to your docker image"""

    name = "urdf_viz"

    def required(self, cliargs):
        return {"curl", "ros_humble"}

    def invoke_after(self, cliargs):
        return {"curl", "ros_humble"}
