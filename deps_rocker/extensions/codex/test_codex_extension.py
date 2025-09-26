from __future__ import annotations

import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


if "em" not in sys.modules:
    em_stub = types.ModuleType("em")

    def expand(template: str, args: dict | None = None):
        return template

    em_stub.expand = expand
    sys.modules["em"] = em_stub

from deps_rocker.extensions.codex.codex import Codex


class TestCodexExtension(unittest.TestCase):
    def setUp(self):
        self.extension = Codex()

    def test_get_docker_args_mounts_codex_home(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            host_home = Path(tmpdir)
            codex_home = host_home / ".codex"
            codex_home.mkdir()

            with mock.patch.dict(os.environ, {"HOME": str(host_home)}, clear=False):
                docker_args = self.extension.get_docker_args({"user_home_dir": "/home/container"})

        expected_mount = f'-v "{codex_home.resolve()}:/home/container/.codex"'
        expected_env = '-e "CODEX_HOME=/home/container/.codex"'

        self.assertIn(expected_mount, docker_args)
        self.assertIn(expected_env, docker_args)

    def test_get_docker_args_no_host_codex(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            host_home = Path(tmpdir)

            with mock.patch.dict(os.environ, {"HOME": str(host_home)}, clear=False):
                docker_args = self.extension.get_docker_args({"user_home_dir": "/home/container"})

        self.assertEqual(docker_args, "")

    def test_get_docker_args_missing_user_home_dir(self):
        with (
            tempfile.TemporaryDirectory() as tmpdir,
            mock.patch("deps_rocker.extensions.codex.codex.logging") as mock_logging,
        ):
            host_home = Path(tmpdir)
            codex_home = host_home / ".codex"
            codex_home.mkdir()

            with mock.patch.dict(os.environ, {"HOME": str(host_home)}, clear=False):
                docker_args = self.extension.get_docker_args({})
            self.assertEqual(docker_args, "")
            mock_logging.warning.assert_called()

    def test_get_docker_args_invalid_user_home_dir(self):
        with (
            tempfile.TemporaryDirectory() as tmpdir,
            mock.patch("deps_rocker.extensions.codex.codex.logging") as mock_logging,
        ):
            host_home = Path(tmpdir)
            codex_home = host_home / ".codex"
            codex_home.mkdir()

            with mock.patch.dict(os.environ, {"HOME": str(host_home)}, clear=False):
                docker_args_none = self.extension.get_docker_args({"user_home_dir": None})
                docker_args_empty = self.extension.get_docker_args({"user_home_dir": ""})
            self.assertEqual(docker_args_none, "")
            self.assertEqual(docker_args_empty, "")
            self.assertGreaterEqual(mock_logging.warning.call_count, 2)
