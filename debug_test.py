#!/usr/bin/env python3

import tempfile
from rocker.core import list_plugins, DockerImageGenerator


class ScriptInjectionExtension:
    """Quick extension for injecting test scripts"""

    def __init__(self, script_content):
        self.script_content = script_content

    def get_name(self):
        return "test_script"

    def get_files(self, cliargs):
        return {"test.sh": self.script_content}

    def get_preamble(self, cliargs):
        return ""

    def get_snippet(self, cliargs):
        return """COPY test.sh /tmp/test.sh
RUN chmod +x /tmp/test.sh
CMD ["/tmp/test.sh"]"""

    def get_user_snippet(self, cliargs):
        return ""


def main():
    # Load debug script content
    with open("/home/ags/projects/deps_rocker/deep_debug_env.sh", "r") as f:
        debug_script = f.read()

    all_plugins = list_plugins()

    if "ros_jazzy" not in all_plugins:
        print("ros_jazzy extension not available")
        return

    # Set up extensions
    cliargs = {"base_image": "testfixture_ros_jazzy_comprehensive", "ros_jazzy": True}

    active_extensions = []

    # Add ros_jazzy extension
    ros_ext = all_plugins["ros_jazzy"]()
    active_extensions.append(ros_ext)

    # Add required dependencies
    for dep in ros_ext.required(cliargs):
        if dep in all_plugins:
            active_extensions.append(all_plugins[dep]())
            cliargs[dep] = True

    # Add debug script
    active_extensions.append(ScriptInjectionExtension(debug_script))
    cliargs["command"] = "/tmp/test.sh"

    # Build and run
    dig = DockerImageGenerator(active_extensions, cliargs, "testfixture_ros_jazzy_comprehensive")

    print("Building Docker image...")
    build_result = dig.build()
    if build_result != 0:
        print(f"Build failed with code {build_result}")
        return

    print("Running Docker container...")
    with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
        run_result = dig.run(console_output_file=tmpfile.name)
        tmpfile.seek(0)
        output = tmpfile.read()
        print(f"Run result: {run_result}")
        print("Container output:")
        print("=" * 50)
        print(output)
        print("=" * 50)


if __name__ == "__main__":
    main()
