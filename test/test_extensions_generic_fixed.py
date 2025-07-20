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
        
        # Filter to only deps_rocker extensions
        for name, plugin_class in all_plugins.items():
            if hasattr(plugin_class, '__module__') and plugin_class.__module__.startswith('deps_rocker'):
                deps_rocker_plugins[name] = plugin_class
        
        return deps_rocker_plugins

    def setUp(self):
        """Get available plugins for each test"""
        self.plugins = self.get_deps_rocker_plugins()
        self.plugin_names = list(self.plugins.keys())

    def _prepare_cliargs(self, extension_name):
        """Prepare cliargs with special handling for specific extensions"""
        cliargs = {
            'base_image': self.base_dockerfile_tag,
            extension_name: True,
        }
        
        # Add special handling for dependencies extension
        if extension_name == 'dependencies':
            cliargs['deps'] = []  # Empty list for testing
            
        return cliargs

    def test_single_extension_builds(self):
        """Test that each extension can build successfully in isolation"""
        if not self.plugin_names:
            self.skipTest("No deps_rocker extensions found")
            
        for extension_name in self.plugin_names:
            with self.subTest(extension=extension_name):
                self._test_single_extension_build(extension_name)

    def _test_single_extension_build(self, extension_name):
        """Helper method to test a single extension build"""
        try:
            extension_class = self.plugins[extension_name]
            active_extensions = [extension_class()]
            cliargs = self._prepare_cliargs(extension_name)
            
            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()
            
            # Clean up the built image
            dig.clear_image()
            
            self.assertEqual(build_result, 0, f"Extension '{extension_name}' failed to build")
            
        except Exception as e:
            self.fail(f"Extension '{extension_name}' raised an exception during build: {e}")

    def test_all_extensions_build_individually(self):
        """Test each extension builds successfully in isolation"""
        if not self.plugin_names:
            self.skipTest("No deps_rocker extensions found")
            
        for extension_name in self.plugin_names:
            with self.subTest(extension=extension_name):
                self._test_single_extension_build(extension_name)

    def test_all_extensions_run_individually(self):
        """Test each extension can run successfully in isolation"""
        if not self.plugin_names:
            self.skipTest("No deps_rocker extensions found")
            
        for extension_name in self.plugin_names:
            with self.subTest(extension=extension_name):
                self._test_single_extension_run(extension_name)

    def _test_single_extension_run(self, extension_name):
        """Helper method to test a single extension run"""
        try:
            extension_class = self.plugins[extension_name]
            active_extensions = [extension_class()]
            cliargs = self._prepare_cliargs(extension_name)
            
            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()
            self.assertEqual(build_result, 0, f"Extension '{extension_name}' failed to build for run test")
            
            # Test running the container
            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                self.assertEqual(run_result, 0, f"Extension '{extension_name}' failed to run")
                
                # Check that we got expected output
                tmpfile.seek(0)
                output = tmpfile.read()
                self.assertIn("Extension test complete", output)
            
            # Clean up the built image
            dig.clear_image()
            
        except Exception as e:
            self.fail(f"Extension '{extension_name}' raised an exception during run: {e}")

    def test_extension_docker_args_format(self):
        """Test that extensions follow proper Docker argument formats"""
        if not self.plugin_names:
            self.skipTest("No deps_rocker extensions found")
            
        for extension_name in self.plugin_names:
            with self.subTest(extension=extension_name):
                extension_class = self.plugins[extension_name]
                extension = extension_class()
                
                # Test basic properties exist
                self.assertTrue(hasattr(extension, 'get_name'))
                self.assertTrue(callable(extension.get_name))
                
                # Test name format (should be valid for docker)
                name = extension.get_name()
                self.assertIsInstance(name, str)
                self.assertGreater(len(name), 0)

    def test_extension_combinations_build(self):
        """Test various combinations of extensions work together"""
        if len(self.plugin_names) < 2:
            self.skipTest("Need at least 2 extensions for combination tests")
            
        # Test pairs of extensions
        for i, ext1 in enumerate(self.plugin_names):
            for ext2 in self.plugin_names[i+1:i+3]:  # Test with a few combinations
                with self.subTest(combination=[ext1, ext2]):
                    self._test_extension_combination([ext1, ext2])

    def _test_extension_combination(self, extension_names):
        """Helper method to test a combination of extensions"""
        try:
            active_extensions = []
            cliargs = {'base_image': self.base_dockerfile_tag}
            
            for ext_name in extension_names:
                ext_class = self.plugins[ext_name]
                active_extensions.append(ext_class())
                cliargs[ext_name] = True
                
                # Add special handling for dependencies extension
                if ext_name == 'dependencies':
                    cliargs['deps'] = []  # Empty list for testing
            
            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()
            
            # Clean up the built image
            dig.clear_image()
            
            self.assertEqual(build_result, 0, f"Extension combination {extension_names} failed to build")
            
        except Exception as e:
            self.fail(f"Extension combination {extension_names} raised an exception: {e}")

    def test_all_extensions_together(self):
        """Test that all extensions can be used together"""
        if not self.plugin_names:
            self.skipTest("No deps_rocker extensions found")
            
        try:
            active_extensions = []
            cliargs = {'base_image': self.base_dockerfile_tag}

            # Enable all extensions
            for ext_name, ext_class in self.plugins.items():
                active_extensions.append(ext_class())
                cliargs[ext_name] = True
                
                # Add special handling for dependencies extension
                if ext_name == 'dependencies':
                    cliargs['deps'] = []  # Empty list for testing

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


if __name__ == "__main__":
    unittest.main()
