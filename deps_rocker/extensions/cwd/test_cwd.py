import unittest
from deps_rocker.extensions.cwd.cwd import CWD, CWDName
from pathlib import Path
import re
import os
import pwd


class TestCWD(unittest.TestCase):
    def test_get_docker_args(self):
        cwd_ext = CWD()
        # Simulate user extension providing home directory
        container_home = pwd.getpwuid(os.getuid()).pw_dir
        cliargs = {"user_home_dir": container_home}
        docker_args = cwd_ext.get_docker_args(cliargs)

        host_cwd = Path.cwd()
        project_name = host_cwd.name
        expected_path = f"{container_home}/{project_name}"
        uid = os.getuid()
        gid = os.getgid()
        expected = f' -u {uid}:{gid} -v "{host_cwd}:{expected_path}:Z" -w "{expected_path}"'
        self.assertEqual(docker_args, expected)

    def test_get_docker_args_no_home(self):
        cwd_ext = CWD()
        cliargs = {}  # No user_home_dir provided
        docker_args = cwd_ext.get_docker_args(cliargs)
        # Should still work with fallback
        self.assertIn(str(Path.cwd()), docker_args)
        self.assertIn("-u", docker_args)
        self.assertIn("-v", docker_args)
        self.assertIn(":Z", docker_args)  # Check for SELinux flag
        self.assertIn("-w", docker_args)

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
