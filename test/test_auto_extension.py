import unittest
import tempfile
import os
from pathlib import Path
from deps_rocker.extensions.auto.auto import Auto


class TestAutoExtension(unittest.TestCase):
    def test_detect_pixi_with_auto_path(self):
        """Test detection of pixi.toml with --auto=/path argument"""

        def setup():
            subdir = Path("subdir")
            subdir.mkdir(exist_ok=True)
            (subdir / "pixi.toml").touch()

        def assertion(deps):
            self.assertIn("pixi", deps)

        original_dir = os.getcwd()
        try:
            os.chdir(self.test_dir)
            setup()
            deps = self.auto.required({"auto": str(Path(self.test_dir) / "subdir")})
            assertion(deps)
        finally:
            os.chdir(original_dir)

    def test_detect_uv_pyproject_with_auto_path(self):
        """Test detection of pyproject.toml for uv with --auto=/path argument"""

        def setup():
            subdir = Path("subdir")
            subdir.mkdir(exist_ok=True)
            (subdir / "pyproject.toml").touch()

        def assertion(deps):
            self.assertIn("uv", deps)

        original_dir = os.getcwd()
        try:
            os.chdir(self.test_dir)
            setup()
            deps = self.auto.required({"auto": str(Path(self.test_dir) / "subdir")})
            assertion(deps)
        finally:
            os.chdir(original_dir)

    def test_detect_uv_poetry_lock(self):
        """Test detection of poetry.lock for uv"""

        def setup():
            Path("poetry.lock").touch()

        def assertion(deps):
            self.assertIn("uv", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_conda_environment_yaml(self):
        """Test detection of environment.yaml for conda"""

        def setup():
            Path("environment.yaml").touch()

        def assertion(deps):
            self.assertIn("conda", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_ccache_cc(self):
        """Test detection of .cc files for ccache"""

        def setup():
            Path("main.cc").touch()

        def assertion(deps):
            self.assertIn("ccache", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_ccache_cxx(self):
        """Test detection of .cxx files for ccache"""

        def setup():
            Path("main.cxx").touch()

        def assertion(deps):
            self.assertIn("ccache", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_ccache_h(self):
        """Test detection of .h files for ccache"""

        def setup():
            Path("header.h").touch()

        def assertion(deps):
            self.assertIn("ccache", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_ccache_c(self):
        """Test detection of .c files for ccache"""

        def setup():
            Path("main.c").touch()

        def assertion(deps):
            self.assertIn("ccache", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_ccache_hxx(self):
        """Test detection of .hxx files for ccache"""

        def setup():
            Path("header.hxx").touch()

        def assertion(deps):
            self.assertIn("ccache", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_uv_pyproject_and_requirements(self):
        """Test detection of pyproject.toml and requirements.txt for overlapping uv triggers"""

        def setup():
            Path("pyproject.toml").touch()
            Path("requirements.txt").touch()

        def assertion(deps):
            self.assertIn("uv", deps)

        self._test_in_dir(setup, assertion)

    def test_detect_conda_and_requirements(self):
        """Test detection of environment.yml and requirements.txt for overlapping conda/uv triggers"""

        def setup():
            Path("environment.yml").touch()
            Path("requirements.txt").touch()

        def assertion(deps):
            self.assertIn("conda", deps)
            self.assertIn("uv", deps)

        self._test_in_dir(setup, assertion)

    # Tests for the auto extension file detection logic

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


    def test_detect_pixi_pyproject_with_section(self):
        """Test detection of pyproject.toml with [tool.pixi] section for pixi"""
        def setup():
            with open("pyproject.toml", "w") as f:
                f.write("[tool.pixi]\nfoo = 'bar'\n")
        def assertion(deps):
            self.assertIn("pixi", deps)
        self._test_in_dir(setup, assertion)

    def test_detect_pixi_pyproject_without_section(self):
        """Test detection of pyproject.toml without [tool.pixi] section for pixi (should not activate)"""
        def setup():
            with open("pyproject.toml", "w") as f:
                f.write("[project]\nname = 'test'\n")
        def assertion(deps):
            self.assertNotIn("pixi", deps)
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
        """Test that empty directory returns empty set (ignoring extensions detected from home directory)"""
        original_dir = os.getcwd()
        try:
            os.chdir(self.test_dir)
            deps = self.auto.required({})
            # Remove extensions that are detected from home directory
            home_extensions = {"claude", "gemini", "nvim", "codex"}
            deps = set(deps) - home_extensions
            self.assertEqual(deps, set())
        finally:
            os.chdir(original_dir)


if __name__ == "__main__":
    unittest.main()
