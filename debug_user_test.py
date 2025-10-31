#!/usr/bin/env python3
import tempfile
import sys

sys.path.append(".")

from rocker.core import DockerImageGenerator
from rocker.core import RockerExtensionManager


class ScriptInjectionExtension:
    def __init__(self, script_content, is_content=True):
        self.script_content = script_content
        self.is_content = is_content

    def get_name(self):
        return "test_script"

    def get_preamble(self, args):  # pylint: disable=unused-argument
        return ""

    def get_snippet(self, args):  # pylint: disable=unused-argument
        return """COPY test.sh /tmp/test.sh
RUN chmod +x /tmp/test.sh
CMD ["/tmp/test.sh"]"""

    def get_user_snippet(self, args):  # pylint: disable=unused-argument
        return ""

    def get_files(self, args):  # pylint: disable=unused-argument
        return {"test.sh": self.script_content}


# Simple debug script to see what user and environment we have
debug_script = """#!/bin/bash
set -e
echo "=== USER AND ENVIRONMENT DEBUG ==="
echo "User: $(whoami)"
echo "UID: $(id -u)"
echo "GID: $(id -g)"
echo "HOME: $HOME"
echo "Current directory: $(pwd)"
echo "ROS_UNDERLAY_ROOT: ${ROS_UNDERLAY_ROOT:-NOT_SET}"
echo "ROS_OVERLAY_ROOT: ${ROS_OVERLAY_ROOT:-NOT_SET}"
echo "Directory listing of home:"
ls -la "$HOME" || echo "Cannot list $HOME"
echo "=== END DEBUG ==="
"""

manager = RockerExtensionManager()
# Build cli args exactly like the test
cliargs = {
    "base_image": "testfixture_ros_jazzy_comprehensive",
    "extension_blacklist": [],
    "strict_extension_selection": False,
    "ros_jazzy": True,
}

active_extensions = manager.get_active_extensions(cliargs)
active_extensions.append(ScriptInjectionExtension(debug_script, is_content=True))
cliargs["command"] = "/tmp/test.sh"

dig = DockerImageGenerator(active_extensions, cliargs, "testfixture_ros_jazzy_comprehensive")
build_result = dig.build()
print(f"Build result: {build_result}")

if build_result == 0:
    with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
        run_result = dig.run(console_output_file=tmpfile.name)
        tmpfile.seek(0)
        output = tmpfile.read()
        print(f"Run result: {run_result}")
        print(f"Container output:\n{output}")
