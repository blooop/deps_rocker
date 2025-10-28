import unittest
import tempfile
import shutil
import yaml
from pathlib import Path
from deps_rocker.extensions.ros_jazzy.ros_jazzy import RosJazzy


class TestRosJazzyExtension(unittest.TestCase):
    """Test ROS Jazzy extension robustness with various .repos file conditions"""

    def setUp(self):
        """Create a temporary directory for testing"""
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        """Clean up temporary directory"""
        shutil.rmtree(self.test_dir)

    def test_empty_repos_files(self):
        """Test that ROS extension handles empty .repos files gracefully"""
        workspace = Path(self.test_dir)
        
        # Create various problematic .repos files
        empty_repos = workspace / "empty.repos"
        empty_repos.write_text("")  # Empty file
        
        comment_only_repos = workspace / "comment_only.repos"
        comment_only_repos.write_text("# Just a comment\n")  # Only comments
        
        null_repos = workspace / "null.repos"
        null_repos.write_text("repositories: null\n")  # Null repositories
        
        # Test ROS extension - should not crash
        ros_ext = RosJazzy()
        files = ros_ext.get_files({"auto": str(workspace)})
        
        # Check that consolidated.repos was created
        self.assertIn("consolidated.repos", files)
        consolidated = yaml.safe_load(files["consolidated.repos"])
        self.assertIsInstance(consolidated, dict)
        self.assertIn("repositories", consolidated)

    def test_valid_repos_file(self):
        """Test that ROS extension processes valid .repos files correctly"""
        workspace = Path(self.test_dir)
        
        # Create a valid .repos file
        valid_repos = workspace / "test.repos"
        valid_repos.write_text("""
repositories:
  test_package:
    type: git
    url: https://github.com/example/test_package.git
    version: main
""")
        
        ros_ext = RosJazzy()
        files = ros_ext.get_files({"auto": str(workspace)})
        
        # Check that the repository was included
        consolidated = yaml.safe_load(files["consolidated.repos"])
        self.assertIn("test_package", consolidated["repositories"])
        self.assertEqual(
            consolidated["repositories"]["test_package"]["url"],
            "https://github.com/example/test_package.git"
        )

    def test_malformed_yaml_repos(self):
        """Test that ROS extension handles malformed YAML in .repos files"""
        workspace = Path(self.test_dir)
        
        # Create a malformed .repos file
        malformed_repos = workspace / "malformed.repos"
        malformed_repos.write_text("repositories:\n  invalid: [\n")  # Unclosed bracket
        
        # Should not crash
        ros_ext = RosJazzy()
        files = ros_ext.get_files({"auto": str(workspace)})
        
        # Should still create consolidated.repos (just without the malformed file)
        self.assertIn("consolidated.repos", files)


if __name__ == "__main__":
    unittest.main()