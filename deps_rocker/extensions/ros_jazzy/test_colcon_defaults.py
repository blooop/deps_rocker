import unittest
import tempfile
import os
import yaml

from deps_rocker.extensions.ros_jazzy.ros_jazzy import RosJazzy


class TestRosJazzyColconDefaults(unittest.TestCase):
    def test_get_files_empy_substitution(self):
        """Test that empy substitution works in get_files method"""

        # Create a mock CLI args object with a get method
        class MockCliArgs:
            def __init__(self):
                self.name = "testuser"
                self.auto = None
                self.base_image = ""

            def get(self, key, default=None):
                return getattr(self, key, default)

        # Instantiate the RosJazzy extension
        ros_jazzy = RosJazzy()

        # Use a temporary directory to simulate a clean environment
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create a sample empy template with substitution variables
            template_content = """@{
ros_ws_root = '/home/' + name + '/overlay'
}@
build:
  base-paths: ["@(ros_ws_root)/src"]
  build-base: "@(ros_ws_root)/build"
  install-base: "@(ros_ws_root)/install"

test:
  build-base: "@(ros_ws_root)/build"
  install-base: "@(ros_ws_root)/install"

'':
  log-base: "@(ros_ws_root)/log"
"""
            # Mock the get_config_file method to return our test template
            ros_jazzy.get_config_file = lambda path: template_content.encode("utf-8")
            ros_jazzy._determine_username = lambda: "testuser"

            # Call get_files with mock CLI args
            files = ros_jazzy.get_files(MockCliArgs())

            # Verify colcon-defaults.yaml is in the files
            self.assertIn("colcon-defaults.yaml", files)

            # Read the processed content
            processed_content = files["colcon-defaults.yaml"]

            # Check that no placeholders remain
            self.assertNotIn("@(", processed_content, "Empy placeholders '@(' still present")
            self.assertNotIn("@{", processed_content, "Empy placeholders '@{' still present")

            # Load and validate the YAML
            processed_yaml = yaml.safe_load(processed_content)

            # Validate specific substitutions
            self.assertEqual(processed_yaml["build"]["base-paths"], ["/home/testuser/overlay/src"])
            self.assertEqual(processed_yaml["build"]["build-base"], "/home/testuser/overlay/build")
            self.assertEqual(
                processed_yaml["build"]["install-base"], "/home/testuser/overlay/install"
            )
            self.assertEqual(processed_yaml["test"]["build-base"], "/home/testuser/overlay/build")
            self.assertEqual(processed_yaml[""]["log-base"], "/home/testuser/overlay/log")


if __name__ == "__main__":
    unittest.main()
