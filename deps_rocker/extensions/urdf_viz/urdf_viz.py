from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class UrdfViz(SimpleRockerExtension):
    """Add the urdf-viz to your docker image"""

    name = "urdf_viz"
    apt_packages = ["libxi6", "libxcursor-dev", "libxrandr-dev", "jq", "ros-humble-xacro"]
    builder_apt_packages = ["curl", "ca-certificates", "jq", "tar"]

    def required(self, cliargs):
        return {"curl", "ros_humble"}

    def invoke_after(self, cliargs):
        return {"curl", "ros_humble"}
