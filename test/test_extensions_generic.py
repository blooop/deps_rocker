
import unittest
import pytest
import io
import tempfile
from rocker.core import DockerImageGenerator, list_plugins, get_docker_client


@pytest.mark.docker

class TestExtensionsGeneric(unittest.TestCase):
    """Simplified tests for deps_rocker extensions"""

    EXTENSIONS_TO_TEST = ["uv", "tzdata"]

    @classmethod
    def setUpClass(cls):
        """Build a simple base image for testing extensions"""
        cls.base_dockerfile_tag = "testfixture_extensions_base"
        cls.base_dockerfile = (
            """
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y coreutils curl && apt-get clean
CMD [\"echo\", \"Extension test complete\"]
"""
        )
        client = get_docker_client()
        iof = io.BytesIO(cls.base_dockerfile.encode())
        im = client.build(fileobj=iof, tag=cls.base_dockerfile_tag)
        for _ in im:
            pass

    @classmethod
    def tearDownClass(cls):
        """Clean up the base test image"""
        client = get_docker_client()
        try:
            client.remove_image(cls.base_dockerfile_tag, force=True)
        except Exception:
            pass

    def setUp(self):
        self.all_plugins = list_plugins()
        self.working_extension_names = self.get_working_extensions()

    def get_working_extensions(self):
        all_plugins = list_plugins()
        working_extensions = []
        for name in self.EXTENSIONS_TO_TEST:
            if name in all_plugins:
                plugin_class = all_plugins[name]
                if hasattr(plugin_class, "__module__") and plugin_class.__module__.startswith("deps_rocker"):
                    working_extensions.append(name)
        return working_extensions

    def run_extension_build_and_test(self, extension_name):
        """Shared logic to build, run, and check an extension"""
        if extension_name not in self.working_extension_names:
            self.skipTest(f"Extension '{extension_name}' not available or not from deps_rocker")

        extension_class = self.all_plugins[extension_name]
        extension_instance = extension_class()

        cliargs = {
            "base_image": self.base_dockerfile_tag,
            extension_name: True,
        }
        if extension_name == "odeps_dependencies":
            cliargs["deps"] = "deps_rocker.deps.yaml"

        required_deps = extension_instance.required(cliargs)
        active_extensions = []
        for dep_name in required_deps:
            if dep_name in self.all_plugins:
                dep_class = self.all_plugins[dep_name]
                active_extensions.append(dep_class())
                cliargs[dep_name] = True
        active_extensions.append(extension_instance)

        dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
        build_result = dig.build()
        self.assertEqual(build_result, 0, f"Extension '{extension_name}' failed to build")

        with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
            run_result = dig.run(console_output_file=tmpfile.name)
            self.assertEqual(run_result, 0, f"Extension '{extension_name}' failed to run")
            tmpfile.seek(0)
            output = tmpfile.read()
            self.assertIn(
                "Extension test complete",
                output,
                f"Extension '{extension_name}' did not produce expected output",
            )
        dig.clear_image()

    def test_uv_extension(self):
        self.run_extension_build_and_test("uv")

    def test_tzdata_extension(self):
        self.run_extension_build_and_test("tzdata")

    def test_all_extensions_together(self):
        if not self.working_extension_names:
            self.skipTest("No working extensions found")
        try:
            active_extensions = []
            cliargs = {"base_image": self.base_dockerfile_tag}
            all_deps = set()
            for ext_name in self.EXTENSIONS_TO_TEST:
                if ext_name in self.all_plugins:
                    extension_class = self.all_plugins[ext_name]
                    extension_instance = extension_class()
                    temp_cliargs = {
                        "base_image": self.base_dockerfile_tag,
                        ext_name: True,
                    }
                    if ext_name == "odeps_dependencies":
                        temp_cliargs["deps"] = "deps_rocker.deps.yaml"
                        cliargs["deps"] = "deps_rocker.deps.yaml"
                    required_deps = extension_instance.required(temp_cliargs)
                    all_deps.update(required_deps)
            for dep_name in all_deps:
                if dep_name in self.all_plugins:
                    dep_class = self.all_plugins[dep_name]
                    active_extensions.append(dep_class())
                    cliargs[dep_name] = True
            for ext_name in self.EXTENSIONS_TO_TEST:
                if ext_name in self.all_plugins:
                    extension_class = self.all_plugins[ext_name]
                    active_extensions.append(extension_class())
                    cliargs[ext_name] = True
            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()
            self.assertEqual(build_result, 0, "All extensions together failed to build")
            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                self.assertEqual(run_result, 0, "All extensions together failed to run")
                tmpfile.seek(0)
                output = tmpfile.read()
                self.assertIn("Extension test complete", output)
            dig.clear_image()
        except Exception as e:
            self.fail(f"All extensions together raised an exception: {e}")


if __name__ == "__main__":
    unittest.main()
