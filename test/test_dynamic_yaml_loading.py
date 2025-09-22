"""
Tests for dynamic YAML extension loading system
"""

import os
import tempfile
import unittest
from pathlib import Path
import shutil

from deps_rocker.dynamic_yaml_loader import DynamicYamlLoader
from deps_rocker.yaml_extension import YamlRockerExtension


class TestDynamicYamlLoading(unittest.TestCase):
    """Test dynamic YAML extension loading functionality"""

    def setUp(self):
        """Set up test fixtures"""
        self.test_dir = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, str(self.test_dir))

    def test_load_simple_yaml_extension(self):
        """Test loading a simple YAML extension"""
        yaml_content = """
name: test_simple
description: "A simple test extension"
apt_packages:
  - curl
  - wget
"""
        yaml_file = self.test_dir / "test_simple.deps_rocker.yaml"
        yaml_file.write_text(yaml_content)

        # Load the extension
        extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)

        # Verify class properties
        self.assertEqual(extension_class.__name__, "TestSimple")
        self.assertEqual(extension_class.name, "test_simple")

        # Create instance and test
        instance = extension_class()
        self.assertEqual(instance.name, "test_simple")
        self.assertEqual(instance.__doc__, "A simple test extension")
        self.assertEqual(instance.apt_packages, ["curl", "wget"])

    def test_load_yaml_extension_with_dependencies(self):
        """Test loading YAML extension with dependencies"""
        yaml_content = """
name: test_deps
class_name: TestDependencies
description: "Extension with dependencies"
apt_packages:
  - htop
depends_on_extension:
  - curl
  - locales
"""
        yaml_file = self.test_dir / "test_deps.deps_rocker.yaml"
        yaml_file.write_text(yaml_content)

        extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)

        self.assertEqual(extension_class.__name__, "TestDependencies")

        instance = extension_class()
        self.assertEqual(instance.depends_on_extension, ("curl", "locales"))
        self.assertEqual(instance.apt_packages, ["htop"])

    def test_load_yaml_extension_with_dockerfile(self):
        """Test loading YAML extension with companion Dockerfile"""
        yaml_content = """
name: test_dockerfile
description: "Extension with Dockerfile"
apt_packages:
  - vim
"""
        dockerfile_content = """
RUN echo "Custom installation step" && \\
    apt-get update && \\
    apt-get install -y custom-package
"""

        yaml_file = self.test_dir / "test_dockerfile.deps_rocker.yaml"
        dockerfile_file = self.test_dir / "test_dockerfile.Dockerfile"

        yaml_file.write_text(yaml_content)
        dockerfile_file.write_text(dockerfile_content)

        extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)

        # Check that dockerfile path is stored
        self.assertEqual(extension_class._dockerfile_path, str(dockerfile_file))

        instance = extension_class()
        self.assertEqual(instance.get_dockerfile_path(), str(dockerfile_file))

    def test_discover_yaml_extensions(self):
        """Test discovering multiple YAML extensions in a directory"""
        # Create multiple extensions
        extensions_data = [
            ("ext1.deps_rocker.yaml", "name: ext1\ndescription: 'Extension 1'"),
            ("ext2.deps_rocker.yaml", "name: ext2\ndescription: 'Extension 2'"),
            ("subdir/ext3.deps_rocker.yaml", "name: ext3\ndescription: 'Extension 3'"),
        ]

        for filename, content in extensions_data:
            file_path = self.test_dir / filename
            file_path.parent.mkdir(parents=True, exist_ok=True)
            file_path.write_text(content)

        # Also create a non-matching file that should be ignored
        (self.test_dir / "regular.yaml").write_text("not: an extension")
        (self.test_dir / "other.txt").write_text("some text")

        # Discover extensions
        extensions = DynamicYamlLoader.discover_yaml_extensions([str(self.test_dir)])

        # Should find exactly 3 extensions
        self.assertEqual(len(extensions), 3)
        self.assertIn("ext1", extensions)
        self.assertIn("ext2", extensions)
        self.assertIn("ext3", extensions)

        # Verify they're proper extension classes
        for name, ext_class in extensions.items():
            self.assertTrue(issubclass(ext_class, YamlRockerExtension))
            instance = ext_class()
            self.assertEqual(instance.name, name)

    def test_yaml_extension_name_inference(self):
        """Test that extension name is inferred from filename if not specified"""
        yaml_content = """
description: "Extension without explicit name"
apt_packages:
  - git
"""
        yaml_file = self.test_dir / "inferred_name.deps_rocker.yaml"
        yaml_file.write_text(yaml_content)

        extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)

        self.assertEqual(extension_class.name, "inferred_name")
        self.assertEqual(extension_class.__name__, "InferredName")

    def test_invalid_yaml_file(self):
        """Test handling of invalid YAML files"""
        yaml_file = self.test_dir / "invalid.deps_rocker.yaml"
        yaml_file.write_text("invalid: yaml: content: [")

        with self.assertRaises(Exception):
            DynamicYamlLoader.load_extension_from_file(yaml_file)

    def test_empty_yaml_file(self):
        """Test handling of empty YAML files"""
        yaml_file = self.test_dir / "empty.deps_rocker.yaml"
        yaml_file.write_text("")

        with self.assertRaises(ValueError):
            DynamicYamlLoader.load_extension_from_file(yaml_file)

    def test_yaml_extension_all_properties(self):
        """Test YAML extension with all supported properties"""
        yaml_content = """
name: full_featured
class_name: FullFeatured
description: "Extension with all features"
apt_packages:
  - curl
  - git
  - vim
depends_on_extension:
  - locales
  - tzdata
empy_args:
  - "--arg1"
  - "value1"
empy_user_args:
  - "--user-arg"
"""
        yaml_file = self.test_dir / "full_featured.deps_rocker.yaml"
        yaml_file.write_text(yaml_content)

        extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)
        instance = extension_class()

        self.assertEqual(instance.name, "full_featured")
        self.assertEqual(instance.__doc__, "Extension with all features")
        self.assertEqual(instance.apt_packages, ["curl", "git", "vim"])
        self.assertEqual(instance.depends_on_extension, ("locales", "tzdata"))
        self.assertEqual(instance.empy_args, ["--arg1", "value1"])
        self.assertEqual(instance.empy_user_args, ["--user-arg"])

    def test_discovery_ignores_duplicate_files(self):
        """Test that discovery doesn't process the same file multiple times"""
        yaml_content = "name: duplicate_test\ndescription: 'Test extension'"
        yaml_file = self.test_dir / "duplicate_test.deps_rocker.yaml"
        yaml_file.write_text(yaml_content)

        # Call discovery multiple times with overlapping paths
        search_paths = [str(self.test_dir), str(self.test_dir)]
        extensions = DynamicYamlLoader.discover_yaml_extensions(search_paths)

        # Should still only find one extension
        self.assertEqual(len(extensions), 1)
        self.assertIn("duplicate_test", extensions)

    def test_dockerfile_path_when_no_dockerfile_exists(self):
        """Test that dockerfile path is None when no companion file exists"""
        yaml_content = "name: no_dockerfile\ndescription: 'Extension without dockerfile'"
        yaml_file = self.test_dir / "no_dockerfile.deps_rocker.yaml"
        yaml_file.write_text(yaml_content)

        extension_class = DynamicYamlLoader.load_extension_from_file(yaml_file)

        self.assertIsNone(extension_class._dockerfile_path)

        instance = extension_class()
        self.assertIsNone(instance.get_dockerfile_path())


if __name__ == "__main__":
    unittest.main()