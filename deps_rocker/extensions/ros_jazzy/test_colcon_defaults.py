import unittest
import pytest
import tempfile
import os
import yaml

from deps_rocker.extensions.ros_jazzy.ros_jazzy import RosJazzy


class TestRosJazzyColconDefaults(unittest.TestCase):
    @pytest.mark.skip(reason="Skipping ROS Jazzy tests")
    def test_get_files_empy_substitution(self):
        """Test that empy substitution works in get_files method using actual config file"""

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

        # Use a temporary directory to simulate a clean environment for repos/packages
        with tempfile.TemporaryDirectory() as tmpdir:
            # Mock the workspace resolution to point to the temp directory
            original_cwd = os.getcwd()
            try:
                os.chdir(tmpdir)

                # Override username for consistent testing
                ros_jazzy._determine_username = lambda: "testuser"  # pylint: disable=protected-access

                # Call get_files with mock CLI args - uses the actual config file
                files = ros_jazzy.get_files(MockCliArgs())

                # Verify colcon-defaults.yaml is in the files
                self.assertIn("colcon-defaults.yaml", files)

                # Read the processed content
                processed_content = files["colcon-defaults.yaml"]

                # Check that no placeholders remain (empy substitution worked)
                self.assertNotIn("@(", processed_content, "Empy placeholders '@(' still present")
                self.assertNotIn("@{", processed_content, "Empy placeholders '@{' still present")

                # Load and validate the YAML
                processed_yaml = yaml.safe_load(processed_content)

                # Validate specific substitutions - should match the pattern /home/{username}/overlay
                self.assertEqual(
                    processed_yaml["build"]["base-paths"], ["/home/testuser/overlay/src"]
                )
                self.assertEqual(
                    processed_yaml["build"]["build-base"], "/home/testuser/overlay/build"
                )
                self.assertEqual(
                    processed_yaml["build"]["install-base"], "/home/testuser/overlay/install"
                )
                self.assertEqual(
                    processed_yaml["test"]["build-base"], "/home/testuser/overlay/build"
                )
                self.assertEqual(processed_yaml["test"]["log-base"], "/home/testuser/overlay/log")
                self.assertEqual(processed_yaml[""]["log-base"], "/home/testuser/overlay/log")
            finally:
                os.chdir(original_cwd)


if __name__ == "__main__":
    unittest.main()
