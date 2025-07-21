import unittest
import pytest
from hypothesis import given, strategies as st, settings
from rocker.core import DockerImageGenerator, list_plugins, get_docker_client
import io
import tempfile


@pytest.mark.docker
class TestExtensionsGeneric(unittest.TestCase):
    """Generic tests for all available extensions using hypothesis"""

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

    def get_deps_rocker_plugins(self):
        """Get only plugins from deps_rocker package"""
        all_plugins = list_plugins()
        deps_rocker_plugins = {}

        # Filter to only deps_rocker extensions, excluding isaac_sim
        for name, plugin_class in all_plugins.items():
            if hasattr(plugin_class, "__module__") and plugin_class.__module__.startswith(
                "deps_rocker"
            ):
                # Skip isaac_sim extensions for testing - they require special setup
                if name.startswith("isaac"):
                    continue
                deps_rocker_plugins[name] = plugin_class

        return deps_rocker_plugins

    def setUp(self):
        """Get available plugins for each test"""
        self.plugins = self.get_deps_rocker_plugins()
        self.plugin_names = list(self.plugins.keys())

    def test_single_extension_builds(self):
        """Test that each extension can build successfully in true isolation (without dependencies)"""
        if not self.plugin_names:
            self.skipTest("No deps_rocker extensions found")

        for extension_name in self.plugin_names:
            with self.subTest(extension=extension_name):
                extension_class = self.plugins[extension_name]
                extension_instance = extension_class()

                # Check if extension has required dependencies
                cliargs = {
                    "base_image": self.base_dockerfile_tag,
                    extension_name: True,
                }

                # Add special cliargs for specific extensions
                if extension_name == "odeps_dependencies":
                    cliargs["deps"] = "deps_rocker.deps.yaml"

                required_deps = extension_instance.required(cliargs)

                # Skip extensions that have required dependencies for isolation test
                if required_deps:
                    self.skipTest(f"Extension '{extension_name}' has dependencies: {required_deps}")

                try:
                    active_extensions = [extension_instance]
                    dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
                    build_result = dig.build()

                    # Extension should build without error
                    self.assertEqual(
                        build_result,
                        0,
                        f"Extension '{extension_name}' failed to build in isolation",
                    )

                    # Clean up the built image
                    dig.clear_image()

                except Exception as e:
                    self.fail(
                        f"Extension '{extension_name}' raised an exception during isolated build: {e}"
                    )

    def test_all_extensions_build_individually(self):
        """Test each extension builds successfully with its dependencies"""
        all_plugins = list_plugins()  # Get all available plugins including rocker ones

        for extension_name in self.plugin_names:
            # Skip isaac_sim extension as it requires special runtime environment
            if extension_name == "isaac_sim":
                continue

            with self.subTest(extension=extension_name):
                try:
                    extension_class = self.plugins[extension_name]
                    extension_instance = extension_class()

                    # Get required dependencies
                    cliargs = {
                        "base_image": self.base_dockerfile_tag,
                        extension_name: True,  # Enable the extension
                    }

                    # Add special cliargs for specific extensions
                    if extension_name == "odeps_dependencies":
                        cliargs["deps"] = "deps_rocker.deps.yaml"

                    required_deps = extension_instance.required(cliargs)

                    # Build list of active extensions including dependencies
                    active_extensions = []

                    # Add required dependencies first
                    for dep_name in required_deps:
                        if dep_name in all_plugins:
                            dep_class = all_plugins[dep_name]
                            active_extensions.append(dep_class())
                            cliargs[dep_name] = True

                    # Add the main extension
                    active_extensions.append(extension_instance)

                    dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
                    build_result = dig.build()

                    # Extension should build without error
                    self.assertEqual(
                        build_result, 0, f"Extension '{extension_name}' failed to build"
                    )

                    # Clean up the built image
                    dig.clear_image()

                except Exception as e:
                    self.fail(f"Extension '{extension_name}' raised an exception during build: {e}")

    def test_all_extensions_run_individually(self):
        """Test each extension can run successfully with its dependencies"""
        all_plugins = list_plugins()  # Get all available plugins including rocker ones

        for extension_name in self.plugin_names:
            # Skip isaac_sim extension as it requires special runtime environment
            if extension_name == "isaac_sim":
                continue

            with self.subTest(extension=extension_name):
                try:
                    extension_class = self.plugins[extension_name]
                    extension_instance = extension_class()

                    # Get required dependencies
                    cliargs = {
                        "base_image": self.base_dockerfile_tag,
                        extension_name: True,  # Enable the extension
                    }

                    # Add special cliargs for specific extensions
                    if extension_name == "odeps_dependencies":
                        cliargs["deps"] = "deps_rocker.deps.yaml"

                    required_deps = extension_instance.required(cliargs)

                    # Build list of active extensions including dependencies
                    active_extensions = []

                    # Add required dependencies first
                    for dep_name in required_deps:
                        if dep_name in all_plugins:
                            dep_class = all_plugins[dep_name]
                            active_extensions.append(dep_class())
                            cliargs[dep_name] = True

                    # Add the main extension
                    active_extensions.append(extension_instance)

                    dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
                    build_result = dig.build()
                    self.assertEqual(build_result, 0)

                    # Try to run the container
                    with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                        run_result = dig.run(console_output_file=tmpfile.name)
                        # Should run without error (exit code 0)
                        self.assertEqual(
                            run_result, 0, f"Extension '{extension_name}' failed to run"
                        )

                        # Check that we got some output
                        tmpfile.seek(0)
                        output = tmpfile.read()
                        self.assertIn(
                            "Extension test complete",
                            output,
                            f"Extension '{extension_name}' did not produce expected output",
                        )

                    # Clean up the built image
                    dig.clear_image()

                except Exception as e:
                    self.fail(f"Extension '{extension_name}' raised an exception during run: {e}")

    @given(st.lists(st.sampled_from(["cwd", "cwd_name"]), min_size=1, max_size=5, unique=True))
    @settings(max_examples=10, deadline=None)
    def test_extension_combinations_build(self, extension_names):
        """Test that random combinations of extensions build successfully using hypothesis"""
        # Filter to only valid extension names
        valid_extension_names = [name for name in extension_names if name in self.plugin_names]
        if not valid_extension_names:
            return

        try:
            active_extensions = []
            cliargs = {"base_image": self.base_dockerfile_tag}

            for ext_name in valid_extension_names:
                if ext_name in self.plugins:
                    extension_class = self.plugins[ext_name]
                    active_extensions.append(extension_class())
                    cliargs[ext_name] = True

            if not active_extensions:
                return

            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()

            # Combined extensions should build without error
            self.assertEqual(
                build_result, 0, f"Extension combination {valid_extension_names} failed to build"
            )

            # Clean up the built image
            dig.clear_image()

        except Exception as e:
            self.fail(f"Extension combination {valid_extension_names} raised an exception: {e}")

    def test_all_extensions_together(self):
        """Test that all extensions can be used together"""
        try:
            active_extensions = []
            cliargs = {"base_image": self.base_dockerfile_tag}

            # Enable all extensions except isaac_sim
            for ext_name, ext_class in self.plugins.items():
                if ext_name == "isaac_sim":
                    continue
                active_extensions.append(ext_class())
                cliargs[ext_name] = True

                # Add special cliargs for specific extensions
                if ext_name == "odeps_dependencies":
                    cliargs["deps"] = "deps_rocker.deps.yaml"

            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()

            # All extensions together should build without error
            self.assertEqual(build_result, 0, "All extensions together failed to build")

            # Try to run with all extensions
            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                self.assertEqual(run_result, 0, "All extensions together failed to run")

                # Check that we got expected output
                tmpfile.seek(0)
                output = tmpfile.read()
                self.assertIn("Extension test complete", output)

            # Clean up the built image
            dig.clear_image()

        except Exception as e:
            self.fail(f"All extensions together raised an exception: {e}")

    # @given(st.text(min_size=1, max_size=20, alphabet=st.characters(whitelist_categories=('Lu', 'Ll', 'Nd', 'Pc'))))
    # @settings(max_examples=20)
    # def test_extension_docker_args_format(self, test_string):
    #     """Test that extension docker args are properly formatted using hypothesis"""
    #     for extension_name in self.plugin_names:
    #         with self.subTest(extension=extension_name):
    #             extension_class = self.plugins[extension_name]
    #             ext = extension_class()

    #             cliargs = {
    #                 'base_image': self.base_dockerfile_tag,
    #                 extension_name: test_string,
    #             }

    #             try:
    #                 docker_args = ext.get_docker_args(cliargs)
    #                 # Docker args should be a string
    #                 self.assertIsInstance(docker_args, str,
    #                                     f"Extension '{extension_name}' returned non-string docker args")

    #                 # Should not contain newlines (would break docker command)
    #                 self.assertNotIn('\n', docker_args,
    #                                f"Extension '{extension_name}' returned docker args with newlines")

    #             except Exception as e:
    #                 # Some extensions might not handle arbitrary input gracefully, which is OK
    #                 pass


if __name__ == "__main__":
    unittest.main()
