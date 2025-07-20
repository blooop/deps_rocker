import unittest
from deps_rocker.cwd import CWD, CWDName
from pathlib import Path
import re
import pytest
from rocker.core import DockerImageGenerator, list_plugins


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


@pytest.mark.docker
class TestCWDIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Build a simple Dockerfile for testing the /workspaces mount
        cls.dockerfile_tag = "testfixture_cwd_mount"
        cls.dockerfile = """
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y coreutils && apt-get clean
CMD [\"sh\", \"-c\", \"echo 'Contents of /workspaces:' && ls -la /workspaces && echo 'Mount test complete'\"]
"""
        import io
        from rocker.core import get_docker_client
        client = get_docker_client()
        iof = io.BytesIO(cls.dockerfile.encode())
        im = client.build(fileobj=iof, tag=cls.dockerfile_tag)

    def test_cwd_mount_present(self):
        # Use the CWD extension
        import tempfile
        import os
        import uuid

        # Create a unique test file in the current working directory
        test_filename = f"testfile_{uuid.uuid4().hex}.txt"
        test_file_content = "mount test file content"
        with open(test_filename, "w") as f:
            f.write(test_file_content)

        try:
            plugins = list_plugins()
            cwd_plugin = plugins['cwd']
            active_extensions = [cwd_plugin()]
            dig = DockerImageGenerator(active_extensions, {}, self.dockerfile_tag)
            self.assertEqual(dig.build(), 0)
            # Run the container and check that /workspaces contains the mounted directory contents
            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                exit_code = dig.run(console_output_file=tmpfile.name)
                self.assertEqual(exit_code, 0)
                tmpfile.seek(0)
                output = tmpfile.read()
                # Check that the mount worked - /workspaces should be accessible and exist
                self.assertIn("Contents of /workspaces:", output)
                self.assertIn("Mount test complete", output)
                # Check that the test file is present in /workspaces
                self.assertIn(test_filename, output)
        finally:
            # Clean up the test file
            if os.path.exists(test_filename):
                os.remove(test_filename)

    @classmethod
    def tearDownClass(cls):
        from rocker.core import get_docker_client
        client = get_docker_client()
        try:
            client.remove_image(cls.dockerfile_tag, force=True)
        except Exception:
            pass


if __name__ == "__main__":
    unittest.main()
