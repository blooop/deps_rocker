import unittest
import tempfile
import os
from pathlib import Path
from deps_rocker.extensions.auto.auto import Auto


class TestAutoExtension(unittest.TestCase):
    """Tests for the auto extension file detection logic"""

    def setUp(self):
        """Create a temporary directory for testing"""
        self.test_dir = tempfile.mkdtemp()
        self.auto = Auto()

    def tearDown(self):
        """Clean up temporary directory"""
        import shutil

        shutil.rmtree(self.test_dir)

    def test_detect_pixi(self):
        """Test detection of pixi.toml"""
        Path(self.test_dir, "pixi.toml").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("pixi", deps)

    def test_detect_uv_pyproject(self):
        """Test detection of pyproject.toml for uv"""
        Path(self.test_dir, "pyproject.toml").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("uv", deps)

    def test_detect_uv_requirements(self):
        """Test detection of requirements.txt for uv"""
        Path(self.test_dir, "requirements.txt").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("uv", deps)

    def test_detect_uv_python_version(self):
        """Test detection of .python-version for uv"""
        Path(self.test_dir, ".python-version").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("uv", deps)

    def test_detect_npm(self):
        """Test detection of package.json for npm"""
        Path(self.test_dir, "package.json").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("npm", deps)

    def test_detect_cargo(self):
        """Test detection of Cargo.toml for cargo"""
        Path(self.test_dir, "Cargo.toml").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("cargo", deps)

    def test_detect_conda(self):
        """Test detection of environment.yml for conda"""
        Path(self.test_dir, "environment.yml").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("conda", deps)

    def test_detect_ros(self):
        """Test detection of package.xml for ros_jazzy"""
        Path(self.test_dir, "package.xml").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("ros_jazzy", deps)

    def test_detect_ccache_cpp(self):
        """Test detection of .cpp files for ccache"""
        Path(self.test_dir, "main.cpp").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("ccache", deps)

    def test_detect_ccache_hpp(self):
        """Test detection of .hpp files for ccache"""
        Path(self.test_dir, "header.hpp").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("ccache", deps)

    def test_detect_multiple(self):
        """Test detection of multiple file types"""
        Path(self.test_dir, "pixi.toml").touch()
        Path(self.test_dir, "package.json").touch()
        Path(self.test_dir, "main.cpp").touch()
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertIn("pixi", deps)
        self.assertIn("npm", deps)
        self.assertIn("ccache", deps)

    def test_no_files_detected(self):
        """Test that empty directory returns empty set"""
        cliargs = {"dir": self.test_dir}
        deps = self.auto.required(cliargs)
        self.assertEqual(deps, set())

    def test_current_dir_fallback(self):
        """Test that it uses current directory when dir is not in cliargs"""
        # Save current directory
        original_dir = os.getcwd()
        try:
            # Change to test directory
            os.chdir(self.test_dir)
            Path(self.test_dir, "pixi.toml").touch()
            cliargs = {}
            deps = self.auto.required(cliargs)
            self.assertIn("pixi", deps)
        finally:
            # Restore original directory
            os.chdir(original_dir)


if __name__ == "__main__":
    unittest.main()
