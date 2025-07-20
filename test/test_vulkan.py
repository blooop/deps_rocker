# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import em
import unittest
import pytest
import os
import grp

from io import BytesIO as StringIO

from rocker.core import DockerImageGenerator
from rocker.core import list_plugins
from rocker.core import get_docker_client


def plugin_load_parser_correctly(plugin):
    """Helper function to test plugin loading"""
    try:
        import argparse

        parser = argparse.ArgumentParser()
        plugin.register_arguments(parser, {})
        return True
    except Exception:
        return False


@pytest.mark.docker
class VulkanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        client = get_docker_client()
        cls.dockerfile_tags = []
        for distro_version in ["jammy", "noble"]:
            dockerfile = """
FROM ubuntu:%(distro_version)s

RUN apt-get update && apt-get install -y --no-install-recommends vulkan-tools mesa-vulkan-drivers && apt-get clean

CMD vulkaninfo --summary
"""
            dockerfile_tag = "testfixture_%s_vulkan" % distro_version
            iof = StringIO((dockerfile % locals()).encode())
            im = client.build(fileobj=iof, tag=dockerfile_tag)
            for _ in im:
                pass
                # print(e)
            cls.dockerfile_tags.append(dockerfile_tag)

    def setUp(self):
        # Work around interference between empy Interpreter
        # stdout proxy and test runner. empy installs a proxy on stdout
        # to be able to capture the information.
        # And the test runner creates a new stdout object for each test.
        # This breaks empy as it assumes that the proxy has persistent
        # between instances of the Interpreter class
        # empy will error with the exception
        # "em.Error: interpreter stdout proxy lost"
        em.Interpreter._wasProxyInstalled = False  # pylint: disable=protected-access

    def test_vulkan_extension_basic(self):
        plugins = list_plugins()
        vulkan_plugin = plugins["vulkan"]
        self.assertEqual(vulkan_plugin.get_name(), "vulkan")
        self.assertTrue(plugin_load_parser_correctly(vulkan_plugin))

        p = vulkan_plugin()
        mock_cliargs = {"base_image": "ubuntu:jammy"}

        # Test snippet generation
        snippet = p.get_snippet(mock_cliargs)
        self.assertIn("vulkan-sdk", snippet)
        self.assertIn("vulkan-tools", snippet)
        self.assertIn("vulkan-validationlayers", snippet)
        self.assertIn("mesa-vulkan-drivers", snippet)
        self.assertIn("XDG_RUNTIME_DIR", snippet)

        # Test Ubuntu 24.04 (noble) specific packages
        mock_cliargs = {"base_image": "ubuntu:noble"}
        snippet = p.get_snippet(mock_cliargs)
        self.assertIn("vulkan-utility-libraries-dev", snippet)

        # Test Ubuntu 22.04 (jammy) specific packages
        mock_cliargs = {"base_image": "ubuntu:jammy"}
        snippet = p.get_snippet(mock_cliargs)
        self.assertIn("vulkan-validationlayers-dev", snippet)

    def test_vulkan_docker_args(self):
        plugins = list_plugins()
        vulkan_plugin = plugins["vulkan"]
        p = vulkan_plugin()
        mock_cliargs = {"base_image": "ubuntu:jammy"}

        docker_args = p.get_docker_args(mock_cliargs)

        # Should always include /dev/dri if it exists
        if os.path.exists("/dev/dri"):
            self.assertIn("--device /dev/dri", docker_args)

        # Should include DRI drivers if they exist
        if os.path.exists("/usr/lib/x86_64-linux-gnu/dri"):
            self.assertIn(
                "-v /usr/lib/x86_64-linux-gnu/dri:/usr/lib/x86_64-linux-gnu/dri:ro", docker_args
            )

        # Should include libdrm if it exists
        if os.path.exists("/usr/share/libdrm"):
            self.assertIn("-v /usr/share/libdrm:/usr/share/libdrm:ro", docker_args)

        # Should include video group if it exists
        try:
            grp.getgrnam("video")
            self.assertIn("--group-add video", docker_args)
        except KeyError:
            pass

        # Should include render group if it exists
        try:
            grp.getgrnam("render")
            self.assertIn("--group-add render", docker_args)
        except KeyError:
            pass

    def test_vulkan_invoke_after(self):
        plugins = list_plugins()
        vulkan_plugin = plugins["vulkan"]
        p = vulkan_plugin()
        mock_cliargs = {}

        # Should invoke after nvidia extension
        invoke_after = p.invoke_after(mock_cliargs)
        self.assertIn("nvidia", invoke_after)

    def test_vulkan_user_snippet(self):
        plugins = list_plugins()
        vulkan_plugin = plugins["vulkan"]
        p = vulkan_plugin()
        mock_cliargs = {"base_image": "ubuntu:jammy"}

        # Test user snippet
        user_snippet = p.get_user_snippet(mock_cliargs)
        self.assertIn("VK_LOADER_DEBUG", user_snippet)
        self.assertIn("VK_LAYER_ENABLES", user_snippet)
        self.assertIn(".local/share/vulkan/settings.d", user_snippet)

    def test_no_vulkan_vulkaninfo(self):
        """Test that vulkaninfo fails without Vulkan extension (with timeout)"""
        import signal

        class TimeoutException(Exception):
            pass

        def handler(signum, frame):
            raise TimeoutException()

        signal.signal(signal.SIGALRM, handler)
        for tag in self.dockerfile_tags:
            dig = DockerImageGenerator([], {}, tag)
            self.assertEqual(dig.build(), 0)
            signal.alarm(30)  # 30 second timeout
            try:
                dig.run()
            except TimeoutException:
                self.fail(f"vulkaninfo (no extension) timed out for {tag}")
            finally:
                signal.alarm(0)

    @pytest.mark.vulkan
    def test_vulkan_vulkaninfo(self):
        """Test that vulkaninfo works with Vulkan extension (with timeout)"""
        import signal

        class TimeoutException(Exception):
            pass

        def handler(signum, frame):
            raise TimeoutException()

        signal.signal(signal.SIGALRM, handler)
        plugins = list_plugins()
        desired_plugins = ["vulkan"]
        active_extensions = [e() for e in plugins.values() if e.get_name() in desired_plugins]
        for tag in self.dockerfile_tags:
            dig = DockerImageGenerator(active_extensions, {}, tag)
            self.assertEqual(dig.build(), 0)
            signal.alarm(30)  # 30 second timeout
            try:
                dig.run()
            except TimeoutException:
                self.fail(f"vulkaninfo (with extension) timed out for {tag}")
            finally:
                signal.alarm(0)

    @pytest.mark.vulkan
    @pytest.mark.x11
    def test_vulkan_with_x11(self):
        """Test Vulkan extension works with X11 extension"""
        plugins = list_plugins()
        desired_plugins = ["vulkan", "x11"]
        active_extensions = [e() for e in plugins.values() if e.get_name() in desired_plugins]

        for tag in self.dockerfile_tags:
            dig = DockerImageGenerator(active_extensions, {}, tag)
            self.assertEqual(dig.build(), 0)
            # Building should succeed with both extensions

    @pytest.mark.vulkan
    @pytest.mark.nvidia
    @pytest.mark.x11
    def test_vulkan_with_nvidia_and_x11(self):
        """Test Vulkan extension works with both NVIDIA and X11 extensions"""
        plugins = list_plugins()
        desired_plugins = ["vulkan", "nvidia", "x11"]
        active_extensions = [e() for e in plugins.values() if e.get_name() in desired_plugins]

        for tag in self.dockerfile_tags:
            dig = DockerImageGenerator(active_extensions, {}, tag)
            self.assertEqual(dig.build(), 0)
            # Building should succeed with all three extensions


if __name__ == "__main__":
    unittest.main()
