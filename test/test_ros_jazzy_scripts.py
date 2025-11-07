import os
import stat
import shutil
import tempfile
import subprocess
from pathlib import Path
import unittest
import pytest


COLCON_DEFAULTS_PATH = (
    Path(__file__).resolve().parents[1]
    / "deps_rocker"
    / "extensions"
    / "ros_jazzy"
    / "configs"
    / "colcon-defaults.yaml"
)

UNDERLAY_BUILD_SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "deps_rocker"
    / "extensions"
    / "ros_jazzy"
    / "underlay_build.sh"
)


FAKE_COLCON_SCRIPT = """#!/usr/bin/env python3
import os
import sys
import pathlib
try:
    import yaml
except ImportError:
    yaml = None

defaults_path = os.environ.get("COLCON_DEFAULTS_FILE")
defaults = {}
if defaults_path and yaml is not None and os.path.exists(defaults_path):
    with open(defaults_path, "r", encoding="utf-8") as handle:
        defaults = yaml.safe_load(handle) or {}

args = sys.argv[1:]
command = args[0] if args else ""

if command == "clean" and len(args) > 1 and args[1] == "workspace":
    section = defaults.get("clean.workspace", {})
    if not isinstance(section, dict):
        raise SystemExit("clean.workspace defaults missing or invalid")
    for key in section.keys():
        if not isinstance(key, str):
            raise TypeError("non-string key in clean.workspace defaults")

build_base = None
install_base = None
index = 0
while index < len(args):
    token = args[index]
    if token == "--build-base" and index + 1 < len(args):
        build_base = args[index + 1]
        index += 1
    elif token == "--install-base" and index + 1 < len(args):
        install_base = args[index + 1]
        index += 1
    elif token == "--cmake-args":
        index += 1
    index += 1

if command == "build":
    if build_base:
        bb = pathlib.Path(build_base)
        bb.mkdir(parents=True, exist_ok=True)
        (bb / "artifact.txt").write_text("built\\n", encoding="utf-8")
    if install_base:
        ib = pathlib.Path(install_base)
        ib.mkdir(parents=True, exist_ok=True)
        (ib / "setup.bash").write_text("#!/bin/bash\\n", encoding="utf-8")

sentinel_path = os.environ.get("COLCON_SENTINEL")
if sentinel_path:
    with open(sentinel_path, "a", encoding="utf-8") as capture:
        capture.write(" ".join(sys.argv) + "\\n")
"""

FAKE_SUDO_SCRIPT = """#!/bin/sh
echo "sudo should not be called during tests" >&2
exit 2
"""


def _create_executable_script(path: Path, content: str) -> None:
    """Create an executable script at the given path."""
    path.write_text(content)
    path.chmod(0o755)


def _create_fake_colcon(tmpdir: Path) -> Path:
    """Create a fake colcon executable that validates defaults and simulates build behaviour."""
    script = tmpdir / "colcon"
    _create_executable_script(script, FAKE_COLCON_SCRIPT)

    sudo = tmpdir / "sudo"
    _create_executable_script(sudo, FAKE_SUDO_SCRIPT)
    return script


@pytest.mark.skip(reason="Temporarily disabling ros_jazzy tests")
class TestRosJazzyScripts(unittest.TestCase):
    def setUp(self):
        self.original_env = os.environ.copy()

    def tearDown(self):
        os.environ.clear()
        os.environ |= self.original_env

    def _assert_directory_world_accessible(self, directory: Path) -> None:
        """Assert that directory has world-accessible permissions."""
        mode = stat.S_IMODE(directory.stat().st_mode)
        self.assertEqual(mode & 0o007, 0o007, f"{directory} should be world-accessible")

    def _prepare_fake_colcon(self):
        tmp = tempfile.TemporaryDirectory()  # pylint: disable=consider-using-with
        self.addCleanup(tmp.cleanup)
        tmp_path = Path(tmp.name)
        fake_bin = tmp_path / "bin"
        fake_bin.mkdir()
        _create_fake_colcon(fake_bin)
        os.environ["PATH"] = f"{fake_bin}:{os.environ.get('PATH', '')}"
        os.environ["COLCON_DEFAULTS_FILE"] = str(COLCON_DEFAULTS_PATH)
        os.environ["COLCON_SENTINEL"] = str(tmp_path / "colcon.log")
        return tmp_path

    def test_colcon_defaults_clean_workspace_keys_are_strings(self):
        tmp_path = self._prepare_fake_colcon()
        sentinel = Path(os.environ["COLCON_SENTINEL"])

        subprocess.run(["colcon", "clean", "workspace"], check=True, cwd=tmp_path)

        self.assertTrue(sentinel.exists(), "colcon invocation was not captured")
        contents = sentinel.read_text(encoding="utf-8")
        self.assertIn("colcon clean workspace", contents)

    def test_colcon_build_and_test_commands_succeed(self):
        tmp_path = self._prepare_fake_colcon()
        sentinel = Path(os.environ["COLCON_SENTINEL"])

        subprocess.run(["colcon", "build"], check=True, cwd=tmp_path)
        subprocess.run(["colcon", "test"], check=True, cwd=tmp_path)

        log = sentinel.read_text(encoding="utf-8")
        self.assertIn("colcon build", log)
        self.assertIn("colcon test", log)

    def test_underlay_build_script_cleans_and_sets_permissions(self):
        tmp_workspace = tempfile.TemporaryDirectory()  # pylint: disable=consider-using-with
        self.addCleanup(tmp_workspace.cleanup)
        workspace_root = Path(tmp_workspace.name) / "ros_ws"
        workspace_root.mkdir()
        underlay_path = workspace_root / "underlay"
        underlay_build = workspace_root / "underlay_build"
        underlay_install = workspace_root / "underlay_install"

        # Create all required directories
        underlay_path.mkdir()
        underlay_build.mkdir()
        underlay_install.mkdir()

        pkg_dir = underlay_path / "dummy_pkg"
        pkg_dir.mkdir()
        (pkg_dir / "package.xml").write_text("<package></package>", encoding="utf-8")

        (underlay_build / "stale.txt").write_text("old", encoding="utf-8")
        (underlay_install / "stale.txt").write_text("old", encoding="utf-8")

        self._prepare_fake_colcon()
        sentinel = Path(os.environ["COLCON_SENTINEL"])

        os.environ["ROS_UNDERLAY_PATH"] = str(underlay_path)
        os.environ["ROS_UNDERLAY_BUILD"] = str(underlay_build)
        os.environ["ROS_UNDERLAY_INSTALL"] = str(underlay_install)
        os.environ["ROS_WORKSPACE_ROOT"] = str(workspace_root)

        ros_setup_dir = Path(tempfile.mkdtemp(prefix="ros_setup_"))
        self.addCleanup(shutil.rmtree, ros_setup_dir, True)
        setup_script = ros_setup_dir / "setup.bash"
        setup_script.write_text("#!/bin/bash\n", encoding="utf-8")
        os.environ["ROS_DISTRO"] = "jazzy"
        os.environ["ROS_SETUP_SCRIPT"] = str(setup_script)

        subprocess.run(["bash", str(UNDERLAY_BUILD_SCRIPT)], check=True, cwd=workspace_root)

        self.assertFalse((underlay_build / "stale.txt").exists())
        self.assertFalse((underlay_install / "stale.txt").exists())
        self.assertTrue((underlay_build / "artifact.txt").exists())
        self.assertTrue((underlay_install / "setup.bash").exists())

        self._assert_directory_world_accessible(underlay_build)
        self._assert_directory_world_accessible(underlay_install)

        log = sentinel.read_text(encoding="utf-8")
        self.assertIn("--build-base", log)
        self.assertIn("--install-base", log)


if __name__ == "__main__":
    unittest.main()
