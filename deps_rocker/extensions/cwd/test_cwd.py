import unittest
from deps_rocker.extensions.cwd.cwd import CWD, CWDName
from pathlib import Path
import re


class TestCWD(unittest.TestCase):
    def test_get_docker_args(self):
        cwd_ext = CWD()
        cliargs = {}
        docker_args = cwd_ext.get_docker_args(cliargs)
        expected = f" -v {Path.cwd()}:/workspaces "
        self.assertEqual(docker_args, expected)

    def test_invoke_after(self):
        cwd_ext = CWD()
        cliargs = {}
        result = cwd_ext.invoke_after(cliargs)
        self.assertIn("user", result)


class TestCWDName(unittest.TestCase):
    def test_get_docker_args(self):
        cwd_name_ext = CWDName()
        cliargs = {}
        expected = f" --name {Path.cwd().stem}"
        docker_args = cwd_name_ext.get_docker_args(cliargs)
        self.assertEqual(docker_args, expected)

    def test_sanitize_container_name_valid(self):
        valid_names = [
            "my-container",
            "container.123",
            "abc-123.def",
            "A.B-C",
        ]
        for name in valid_names:
            sanitized = CWDName.sanitize_container_name(name)
            self.assertTrue(re.match(r"^[a-zA-Z0-9.-]+$", sanitized))
            self.assertEqual(sanitized, name)

    def test_sanitize_container_name_invalid(self):
        invalid_names = [
            "my container!@#",
            "***",
            "   ",
            "!!!abc!!!",
        ]
        for name in invalid_names:
            if re.sub(r"[^a-zA-Z0-9.-]", "-", name).strip("-"):
                sanitized = CWDName.sanitize_container_name(name)
                self.assertTrue(re.match(r"^[a-zA-Z0-9.-]+$", sanitized))
            else:
                with self.assertRaises(ValueError):
                    CWDName.sanitize_container_name(name)


if __name__ == "__main__":
    unittest.main()
