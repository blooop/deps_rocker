from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class RosHumble(SimpleRockerExtension):
    """Adds ros-humble to your docker container"""
    
    name = "ros_humble"

    def invoke_after(self, cliargs):
        return set(["vcstool"])

    def required(self, cliargs):
        return set(["vcstool"])

 