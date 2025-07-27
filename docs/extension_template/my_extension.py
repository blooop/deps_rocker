from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class MyExtension(SimpleRockerExtension):
    """My custom extension description"""

    name = "my_extension"

    def required(self, cliargs):
        """List any required extensions that should be installed first"""
        return {"curl"}  # Example dependency

    def invoke_after(self, cliargs):
        """List any extensions that should be run after this one"""
        return {"user"}  # Example: run after user setup
