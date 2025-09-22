"""
Integration tests for dynamic YAML extension system with rocker
"""

import os
import tempfile
import unittest
from pathlib import Path
import shutil

from deps_rocker.dynamic_yaml_loader import DynamicYamlLoader
from deps_rocker.integrated_loader import IntegratedExtensionLoader


class TestDynamicIntegration(unittest.TestCase):
    """Test integration of dynamic YAML system with deps_rocker"""

    def setUp(self):
        """Set up test fixtures"""
        self.test_dir = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, str(self.test_dir))

    def test_full_extension_with_dockerfile(self):
        """Test a complete extension with YAML config and Dockerfile"""
        # Create YAML config
        yaml_content = """
name: integration_test
description: "Full integration test extension"
apt_packages:
  - curl
  - htop
depends_on_extension:
  - locales
"""

        # Create Dockerfile
        dockerfile_content = """
# Custom installation steps for integration test
RUN echo "Running integration test installation..." && \\
    apt-get update && \\
    echo "Installation complete"
"""

        yaml_file = self.test_dir / "integration_test.deps_rocker.yaml"
        dockerfile_file = self.test_dir / "integration_test.Dockerfile"

        yaml_file.write_text(yaml_content)
        dockerfile_file.write_text(dockerfile_content)

        # Load the extension
        extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)

        # Verify all properties are set correctly
        self.assertEqual(extension_class.__name__, "IntegrationTest")
        self.assertEqual(extension_class.name, "integration_test")
        self.assertEqual(extension_class._dockerfile_path, str(dockerfile_file))

        # Create instance and verify functionality
        instance = extension_class()
        self.assertEqual(instance.name, "integration_test")
        self.assertEqual(instance.__doc__, "Full integration test extension")
        self.assertEqual(instance.apt_packages, ["curl", "htop"])
        self.assertEqual(instance.depends_on_extension, ("locales",))
        self.assertEqual(instance.get_dockerfile_path(), str(dockerfile_file))

        # Verify it's a proper rocker extension
        from deps_rocker.simple_rocker_extension import SimpleRockerExtension
        self.assertIsInstance(instance, SimpleRockerExtension)

    def test_integrated_loader_functionality(self):
        """Test the integrated loader that combines all extension sources"""
        # Create a test extension
        yaml_content = """
name: loader_test
description: "Test for integrated loader"
apt_packages:
  - vim
"""
        yaml_file = self.test_dir / "loader_test.deps_rocker.yaml"
        yaml_file.write_text(yaml_content)

        # Test integrated loader with specific search path
        # We need to temporarily override the search paths
        original_discover = DynamicYamlLoader.discover_yaml_extensions

        def mock_discover(search_paths=None):
            return original_discover([str(self.test_dir)])

        DynamicYamlLoader.discover_yaml_extensions = staticmethod(mock_discover)

        try:
            extensions = IntegratedExtensionLoader.get_all_extensions()
            self.assertIn("loader_test", extensions)

            # Test getting specific extension
            ext_class = IntegratedExtensionLoader.get_extension("loader_test")
            self.assertEqual(ext_class.__name__, "LoaderTest")

            instance = ext_class()
            self.assertEqual(instance.name, "loader_test")
        finally:
            # Restore original function
            DynamicYamlLoader.discover_yaml_extensions = original_discover

    def test_real_world_extension_example(self):
        """Test a real-world example extension"""
        # Create a realistic development tools extension
        yaml_content = """
name: dev_tools
class_name: DevTools
description: "Essential development tools for container"
apt_packages:
  - git
  - curl
  - vim
  - htop
  - tree
  - jq
  - unzip
depends_on_extension:
  - locales
  - tzdata
"""

        dockerfile_content = """
# Install additional development tools
RUN apt-get update && \\
    apt-get install -y \\
        build-essential \\
        cmake \\
        pkg-config \\
        && \\
    apt-get clean && \\
    rm -rf /var/lib/apt/lists/*

# Set up development environment
ENV EDITOR=vim
ENV PAGER=less
"""

        yaml_file = self.test_dir / "dev_tools.deps_rocker.yaml"
        dockerfile_file = self.test_dir / "dev_tools.Dockerfile"

        yaml_file.write_text(yaml_content)
        dockerfile_file.write_text(dockerfile_content)

        # Load and test the extension
        extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)
        instance = extension_class()

        # Verify all properties
        self.assertEqual(instance.name, "dev_tools")
        self.assertEqual(instance.__class__.__name__, "DevTools")
        self.assertIn("git", instance.apt_packages)
        self.assertIn("curl", instance.apt_packages)
        self.assertIn("locales", instance.depends_on_extension)
        self.assertTrue(dockerfile_file.exists())
        self.assertEqual(instance.get_dockerfile_path(), str(dockerfile_file))

    def test_multiple_extensions_discovery(self):
        """Test discovering multiple extensions in the same directory"""
        extensions_configs = [
            ("python_dev.deps_rocker.yaml", """
name: python_dev
description: "Python development environment"
apt_packages:
  - python3-pip
  - python3-venv
"""),
            ("node_dev.deps_rocker.yaml", """
name: node_dev
description: "Node.js development environment"
apt_packages:
  - nodejs
  - npm
depends_on_extension:
  - curl
"""),
            ("rust_dev.deps_rocker.yaml", """
name: rust_dev
description: "Rust development environment"
depends_on_extension:
  - curl
""")
        ]

        # Create all extensions
        for filename, content in extensions_configs:
            (self.test_dir / filename).write_text(content)

        # Create Dockerfile for rust_dev
        rust_dockerfile = self.test_dir / "rust_dev.Dockerfile"
        rust_dockerfile.write_text("""
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
""")

        # Discover all extensions
        extensions = DynamicYamlLoader.discover_yaml_extensions([str(self.test_dir)])

        # Verify all were found
        self.assertEqual(len(extensions), 3)
        expected_names = {"python_dev", "node_dev", "rust_dev"}
        self.assertEqual(set(extensions.keys()), expected_names)

        # Verify specific properties
        python_ext = extensions["python_dev"]()
        self.assertIn("python3-pip", python_ext.apt_packages)

        node_ext = extensions["node_dev"]()
        self.assertIn("curl", node_ext.depends_on_extension)

        rust_ext = extensions["rust_dev"]()
        self.assertIsNotNone(rust_ext.get_dockerfile_path())

    def test_extension_error_handling(self):
        """Test that the system handles various error conditions gracefully"""
        # Test nonexistent extension
        with self.assertRaises(ImportError):
            IntegratedExtensionLoader.get_extension("nonexistent_extension")

        # Test discovery with nonexistent paths
        extensions = DynamicYamlLoader.discover_yaml_extensions(["/nonexistent/path"])
        self.assertEqual(len(extensions), 0)

        # Test malformed YAML is handled gracefully in discovery
        bad_yaml_file = self.test_dir / "bad.deps_rocker.yaml"
        bad_yaml_file.write_text("invalid: yaml: content: [")

        # Discovery should continue despite bad file
        extensions = DynamicYamlLoader.discover_yaml_extensions([str(self.test_dir)])
        # Should not find any extensions (bad one is skipped)
        self.assertEqual(len(extensions), 0)


if __name__ == "__main__":
    unittest.main()