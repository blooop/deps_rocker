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

    def _test_in_dir(self, setup_func, assertion_func):
        """Helper to run a test in the test directory"""
        original_dir = os.getcwd()
        try:
            os.chdir(self.test_dir)
            setup_func()
            deps = self.auto.required({})
            assertion_func(deps)
        finally:
            os.chdir(original_dir)

    def test_detect_pixi(self):
        """Test detection of pixi.toml"""

        def setup():
            Path("pixi.toml").touch()

        def assertion(deps):
            self.assertIn("pixi", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_uv_pyproject(self):
        """Test detection of pyproject.toml for uv"""

        def setup():
            Path("pyproject.toml").touch()

        def assertion(deps):
            self.assertIn("uv", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_uv_requirements(self):
        """Test detection of requirements.txt for uv"""

        def setup():
            Path("requirements.txt").touch()

        def assertion(deps):
            self.assertIn("uv", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_uv_python_version(self):
        """Test detection of .python-version for uv"""

        def setup():
            Path(".python-version").touch()

        def assertion(deps):
            self.assertIn("uv", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_npm(self):
        """Test detection of package.json for npm"""

        def setup():
            Path("package.json").touch()

        def assertion(deps):
            self.assertIn("npm", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_cargo(self):
        """Test detection of Cargo.toml for cargo"""

        def setup():
            Path("Cargo.toml").touch()

        def assertion(deps):
            self.assertIn("cargo", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_conda(self):
        """Test detection of environment.yml for conda"""

        def setup():
            Path("environment.yml").touch()

        def assertion(deps):
            self.assertIn("conda", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_ros(self):
        """Test detection of package.xml for ros_jazzy"""

        def setup():
            Path("package.xml").touch()

        def assertion(deps):
            self.assertIn("ros_jazzy", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_ccache_cpp(self):
        """Test detection of .cpp files for ccache"""

        def setup():
            Path("main.cpp").touch()

        def assertion(deps):
            self.assertIn("ccache", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_ccache_hpp(self):
        """Test detection of .hpp files for ccache"""

        def setup():
            Path("header.hpp").touch()

        def assertion(deps):
            self.assertIn("ccache", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_multiple(self):
        """Test detection of multiple file types"""

        def setup():
            Path("pixi.toml").touch()
            Path("package.json").touch()
            Path("main.cpp").touch()

        def assertion(deps):
            self.assertIn("pixi", deps)
            self.assertIn("npm", deps)
            self.assertIn("ccache", deps)

        self._test_in_dir(setup, assertion)

    def test_no_files_detected(self):
        """Test that empty directory returns empty set"""
        original_dir = os.getcwd()
        try:
            os.chdir(self.test_dir)
            deps = self.auto.required({})
            self.assertEqual(deps, set())
        finally:
            os.chdir(original_dir)


if __name__ == "__main__":
    unittest.main()
