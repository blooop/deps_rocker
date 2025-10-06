from deps_rocker.extensions.ros_generic.ros_generic import RosGeneric


class RosJazzy(RosGeneric):
    """Install ROS Jazzy for development (legacy entrypoint, uses generic class)"""

    name = "ros_jazzy"
    default_ros_distro = "jazzy"

    def _get_pkg(self):
        # Use ros_generic package for config files
        return "deps_rocker.extensions.ros_generic"

    def get_and_expand_empy_template(self, cliargs, empy_args=None, snippet_prefix=""):
        """Override to use ros_generic template files instead of ros_jazzy ones"""
        # Temporarily change name to ros_generic for template discovery
        original_name = self.name
        self.name = "ros_generic"
        try:
            result = super().get_and_expand_empy_template(cliargs, empy_args, snippet_prefix)
        finally:
            # Restore original name
            self.name = original_name
        return result
