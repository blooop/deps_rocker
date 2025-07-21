import unittest
import pytest
from hypothesis import given, strategies as st, settings
from rocker.core import DockerImageGenerator, list_plugins, get_docker_client
import io
import tempfile


@pytest.mark.docker
class TestExtensionsGeneric(unittest.TestCase):
    """Simplified tests for deps_rocker extensions"""

    @classmethod
    def setUpClass(cls):
        """Build a simple base image for testing extensions"""
        cls.base_dockerfile_tag = "testfixture_extensions_base"
        cls.base_dockerfile = """
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y coreutils curl && apt-get clean
CMD ["echo", "Extension test complete"]
"""
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

    def get_working_extensions(self):
        """Get list of working extensions, only testing uv extension"""
        all_plugins = list_plugins()
        working_extensions = []
        
        # Only test the uv extension
        extension_names = ["uv"]
        
        for name in extension_names:
            if name in all_plugins:
                plugin_class = all_plugins[name]
                # Verify it's from deps_rocker package
                if hasattr(plugin_class, "__module__") and plugin_class.__module__.startswith("deps_rocker"):
                    working_extensions.append(name)
        
        return working_extensions

    def setUp(self):
        """Get available plugins for each test"""
        self.all_plugins = list_plugins()
        self.working_extension_names = self.get_working_extensions()

    @given(st.data())
    @settings(max_examples=20, deadline=None)
    def test_extensions_in_isolation(self, data):
        """Test each extension individually with its dependencies"""
        if not self.working_extension_names:
            self.skipTest("No working extensions found")

        # Use hypothesis to sample an extension from the list
        extension_name = data.draw(st.sampled_from(self.working_extension_names))
        
        try:
            extension_class = self.all_plugins[extension_name]
            extension_instance = extension_class()

            # Setup cliargs
            cliargs = {
                "base_image": self.base_dockerfile_tag,
                extension_name: True,
            }

            # Add special cliargs for specific extensions
            if extension_name == "odeps_dependencies":
                cliargs["deps"] = "deps_rocker.deps.yaml"

            # Get required dependencies
            required_deps = extension_instance.required(cliargs)

            # Build list of active extensions including dependencies
            active_extensions = []

            # Add required dependencies first
            for dep_name in required_deps:
                if dep_name in self.all_plugins:
                    dep_class = self.all_plugins[dep_name]
                    active_extensions.append(dep_class())
                    cliargs[dep_name] = True

            # Add the main extension
            active_extensions.append(extension_instance)

            # Build and test
            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()

            self.assertEqual(
                build_result, 0, f"Extension '{extension_name}' failed to build"
            )

            # Test run
            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                self.assertEqual(
                    run_result, 0, f"Extension '{extension_name}' failed to run"
                )

                # Check output
                tmpfile.seek(0)
                output = tmpfile.read()
                self.assertIn(
                    "Extension test complete",
                    output,
                    f"Extension '{extension_name}' did not produce expected output",
                )

            # Clean up
            dig.clear_image()

        except Exception as e:
            self.fail(f"Extension '{extension_name}' raised an exception: {e}")

    @given(st.sampled_from(["all_extensions_together"]))
    @settings(max_examples=1, deadline=None)
    def test_all_extensions_together(self, _):
        """Test all working extensions together"""
        if not self.working_extension_names:
            self.skipTest("No working extensions found")

        try:
            active_extensions = []
            cliargs = {"base_image": self.base_dockerfile_tag}
            
            # Collect all dependencies first
            all_deps = set()
            for ext_name in self.working_extension_names:
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

            # Add all dependencies first
            for dep_name in all_deps:
                if dep_name in self.all_plugins:
                    dep_class = self.all_plugins[dep_name]
                    active_extensions.append(dep_class())
                    cliargs[dep_name] = True

            # Add all main extensions
            for ext_name in self.working_extension_names:
                if ext_name in self.all_plugins:
                    extension_class = self.all_plugins[ext_name]
                    active_extensions.append(extension_class())
                    cliargs[ext_name] = True

            # Build and test
            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()

            self.assertEqual(build_result, 0, "All extensions together failed to build")

            # Test run
            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                self.assertEqual(run_result, 0, "All extensions together failed to run")

                # Check output
                tmpfile.seek(0)
                output = tmpfile.read()
                self.assertIn("Extension test complete", output)

            # Clean up
            dig.clear_image()

        except Exception as e:
            self.fail(f"All extensions together raised an exception: {e}")


if __name__ == "__main__":
    unittest.main()
