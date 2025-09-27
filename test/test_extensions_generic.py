import unittest
import pytest
import io
import tempfile
from rocker.core import DockerImageGenerator, list_plugins, get_docker_client
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


@pytest.mark.docker
class TestExtensionsGeneric(unittest.TestCase):
    """Simplified tests for deps_rocker extensions"""

    EXTENSIONS_TO_TEST = [
        "uv",
        "tzdata",
        "curl",
        "git_clone",
        "locales",
        "neovim",
        "pixi",
        # "urdf_viz",
        "fzf",
        "cwd",
        "claude",
        "codex",
        "npm",
        "gemini",
        "conda",
        # "isaac_sim",
    ]

    @classmethod
    def setUpClass(cls):
        """Build a simple base image for testing extensions"""
        cls.base_dockerfile_tag = "testfixture_extensions_base"
        cls.base_dockerfile = """
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y coreutils curl && apt-get clean
CMD [\"echo\", \"Extension test complete\"]
"""
        client = get_docker_client()
        iof = io.BytesIO(cls.base_dockerfile.encode())
        im = client.build(fileobj=iof, tag=cls.base_dockerfile_tag)
        for _ in im:
            pass

    @classmethod
    def tearDownClass(cls):
        """Clean up the base test image"""
        client = get_docker_client()
        try:
            client.remove_image(cls.base_dockerfile_tag, force=True)
        except Exception:
            pass

    def setUp(self):
        self.all_plugins = list_plugins()
        self.working_extension_names = self.get_working_extensions()

    def get_working_extensions(self):
        all_plugins = list_plugins()
        working_extensions = []
        for name in self.EXTENSIONS_TO_TEST:
            if name in all_plugins:
                plugin_class = all_plugins[name]
                if hasattr(plugin_class, "__module__") and plugin_class.__module__.startswith(
                    "deps_rocker"
                ):
                    working_extensions.append(name)
        return working_extensions

    def run_extension_build_and_test(self, extension_name):
        """Shared logic to build, run, and check an extension, including running test.sh if present"""
        if extension_name not in self.working_extension_names:
            self.skipTest(f"Extension '{extension_name}' not available or not from deps_rocker")

        extension_class = self.all_plugins[extension_name]
        extension_instance = extension_class()

        cliargs = {
            "base_image": self.base_dockerfile_tag,
            extension_name: True,
        }
        if extension_name == "odeps_dependencies":
            cliargs["deps"] = "deps_rocker.deps.yaml"

        required_deps = extension_instance.required(cliargs)
        active_extensions = []
        for dep_name in required_deps:
            if dep_name in self.all_plugins:
                dep_class = self.all_plugins[dep_name]
                active_extensions.append(dep_class())
                cliargs[dep_name] = True
        active_extensions.append(extension_instance)

        import os

        test_sh_path = f"deps_rocker/extensions/{extension_name}/test.sh"
        if os.path.isfile(test_sh_path):
            # Add the test script extension as the last extension
            active_extensions.append(ScriptInjectionExtension(test_sh_path))
            cliargs["command"] = "/tmp/test.sh"

        dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
        build_result = dig.build()
        self.assertEqual(build_result, 0, f"Extension '{extension_name}' failed to build")

        with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
            run_result = dig.run(console_output_file=tmpfile.name)
            tmpfile.seek(0)
            output = tmpfile.read()
            print(f"DEBUG: run_result={run_result}\nContainer output:\n{output}")
            self.assertEqual(
                run_result, 0, f"Extension '{extension_name}' failed to run. Output: {output}"
            )
            # If test.sh exists, just rely on run_result for pass/fail, no output string checks
        dig.clear_image()

    def test_uv_extension(self):
        self.run_extension_build_and_test("uv")

    def test_tzdata_extension(self):
        self.run_extension_build_and_test("tzdata")

    def test_curl_extension(self):
        self.run_extension_build_and_test("curl")

    def test_git_clone_extension(self):
        self.run_extension_build_and_test("git_clone")

    def test_locales_extension(self):
        self.run_extension_build_and_test("locales")

    def test_neovim_extension(self):
        self.run_extension_build_and_test("neovim")

    def test_pixi_extension(self):
        self.run_extension_build_and_test("pixi")

    # def test_urdf_viz_extension(self):
    #     self.run_extension_build_and_test("urdf_viz")

    def test_fzf_extension(self):
        self.run_extension_build_and_test("fzf")

    def test_cwd_extension(self):
        self.run_extension_build_and_test("cwd")

    def test_claude_extension(self):
        self.run_extension_build_and_test("claude")

    def test_codex_extension(self):
        self.run_extension_build_and_test("codex")

    def test_npm_extension(self):
        self.run_extension_build_and_test("npm")

    def test_gemini_extension(self):
        self.run_extension_build_and_test("gemini")

    def test_conda_extension(self):
        self.run_extension_build_and_test("conda")

    # def test_isaac_sim_extension(self):
    #     self.run_extension_build_and_test("isaac_sim")

    def test_z_all_extensions_together(self):
        if not self.working_extension_names:
            self.skipTest("No working extensions found")
        try:
            active_extensions = []
            cliargs = {"base_image": self.base_dockerfile_tag}
            all_deps = set()
            for ext_name in self.EXTENSIONS_TO_TEST:
                if ext_name in self.all_plugins:
                    extension_class = self.all_plugins[ext_name]
                    extension_instance = extension_class()
                    temp_cliargs = {
                        "base_image": self.base_dockerfile_tag,
                        ext_name: True,
                    }
                    if ext_name == "odeps_dependencies":
                        temp_cliargs["deps"] = "deps_rocker.deps.yaml"
                        cliargs["deps"] = "deps_rocker.deps.yaml"
                    required_deps = extension_instance.required(temp_cliargs)
                    all_deps.update(required_deps)
            for dep_name in all_deps:
                if dep_name in self.all_plugins:
                    dep_class = self.all_plugins[dep_name]
                    active_extensions.append(dep_class())
                    cliargs[dep_name] = True
            for ext_name in self.EXTENSIONS_TO_TEST:
                if ext_name in self.all_plugins:
                    extension_class = self.all_plugins[ext_name]
                    active_extensions.append(extension_class())
                    cliargs[ext_name] = True
            dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
            build_result = dig.build()
            self.assertEqual(build_result, 0, "All extensions together failed to build")
            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                self.assertEqual(run_result, 0, "All extensions together failed to run")
                tmpfile.seek(0)
                output = tmpfile.read()
                self.assertIn("Extension test complete", output)
            dig.clear_image()
        except Exception as e:
            self.fail(f"All extensions together raised an exception: {e}")


class ScriptInjectionExtension(SimpleRockerExtension):
    """Injects a test.sh script into the Docker image and runs it as the final step."""

    name = "test_script"

    def __init__(self, script_path):
        self.script_path = script_path
        self.context_name = "test.sh"

    def get_snippet(self, cliargs):
        return f'COPY {self.context_name} /tmp/test.sh\nRUN chmod +x /tmp/test.sh\nCMD ["/tmp/test.sh"]'

    def get_files(self, cliargs):
        with open(self.script_path, "r", encoding="utf-8") as f:
            content = f.read()
        if not content.lstrip().startswith("#!/"):
            raise RuntimeError(
                f"Error: test.sh script '{self.script_path}' is missing a shebang (e.g., #!/bin/bash) at the top."
            )
        return {self.context_name: content}


if __name__ == "__main__":
    unittest.main()
