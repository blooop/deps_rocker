import io
import tempfile
import unittest
import pytest

from rocker.core import DockerImageGenerator, list_plugins, get_docker_client
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class ScriptInjectionExtension(SimpleRockerExtension):
    name = "test_script"

    def __init__(self, script_path: str):
        self.script_path = script_path
        self.context_name = "test.sh"

    def get_snippet(self, cliargs):
        return 'COPY {0} /tmp/test.sh\nRUN chmod +x /tmp/test.sh\nCMD ["/tmp/test.sh"]'.format(
            self.context_name
        )

    def get_files(self, cliargs):
        with open(self.script_path, "r", encoding="utf-8") as f:
            content = f.read()
        if not content.lstrip().startswith("#!/"):
            raise RuntimeError(f"Error: test.sh script '{self.script_path}' is missing a shebang.")
        return {self.context_name: content}


@pytest.mark.docker
@pytest.mark.claude
class TestClaudeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base_dockerfile_tag = "testfixture_claude_base"
        cls.base_dockerfile = """
FROM ubuntu:22.04
RUN apt-get update && apt-get install -y coreutils curl && apt-get clean
CMD [\"echo\", \"Claude integration test complete\"]
"""
        client = get_docker_client()
        iof = io.BytesIO(cls.base_dockerfile.encode())
        im = client.build(fileobj=iof, tag=cls.base_dockerfile_tag)
        for _ in im:
            pass

    @classmethod
    def tearDownClass(cls):
        client = get_docker_client()
        try:
            client.remove_image(cls.base_dockerfile_tag, force=True)
        except Exception:
            pass

    def test_claude_config_mount_and_env(self):
        all_plugins = list_plugins()
        assert "claude" in all_plugins, "Claude extension not registered"
        claude_ext = all_plugins["claude"]()

        cliargs = {"base_image": self.base_dockerfile_tag, "claude": True}
        # Include required deps
        active_exts = []
        for dep in claude_ext.required(cliargs):
            if dep in all_plugins:
                active_exts.append(all_plugins[dep]())
                cliargs[dep] = True
        active_exts.append(claude_ext)

        # Inject the claude test script which prints envs + checks
        test_sh_path = "deps_rocker/extensions/claude/test.sh"
        active_exts.append(ScriptInjectionExtension(test_sh_path))
        cliargs["command"] = "/tmp/test.sh"

        dig = DockerImageGenerator(active_exts, cliargs, self.base_dockerfile_tag)
        build_result = dig.build()
        self.assertEqual(build_result, 0, "Failed to build claude image")
        try:
            with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
                run_result = dig.run(console_output_file=tmpfile.name)
                tmpfile.seek(0)
                output = tmpfile.read()
                print(output)
                self.assertEqual(run_result, 0, f"claude run failed: {output}")
                # Basic confirmations from the script output
                self.assertIn("Using Claude config at:", output)
                self.assertIn("claude is installed and working", output)
                self.assertIn("No stale root Claude config found", output)
        finally:
            dig.clear_image()
