import unittest
import pytest
import io
import tempfile
import importlib
import importlib.util
import sys
from pathlib import Path
from pathlib import Path
from contextlib import contextmanager, nullcontext
from rocker.core import DockerImageGenerator, list_plugins, get_docker_client
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


@pytest.mark.docker
class TestExtensionsGeneric(unittest.TestCase):
    """Simplified tests for deps_rocker extensions"""

    EXTENSIONS_TO_TEST = [
        "nvim",
        "uv",
        "tzdata",
        "curl",
        "git_clone",
        "locales",
        "nvim",
        "pixi",
        # "urdf_viz",
        "fzf",
        # "gitui",
        "lazygit",
        "cwd",
        "ccache",
        "claude",
        "codex",
        "npm",
        "cargo",
        "gemini",
        "jquery",
        # "spec_kit",
        # "palanteer", #very slow
        "conda",
        # "isaac_sim",
        "foxglove",
        # "ros_jazzy",
        "auto",
    ]

    @classmethod
    def setUpClass(cls):
        """Build a simple base image for testing extensions"""
        cls.base_dockerfile_tag = "testfixture_extensions_base"
        cls.base_dockerfile = """
FROM ubuntu:24.04
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
        self.all_plugins = self._discover_plugins()
        self.working_extension_names = self.get_working_extensions()

    def get_working_extensions(self):
        working_extensions = []
        for name in self.EXTENSIONS_TO_TEST:
            if name in self.all_plugins:
                plugin_class = self.all_plugins[name]
                if hasattr(plugin_class, "__module__") and plugin_class.__module__.startswith(
                    "deps_rocker"
                ):
                    working_extensions.append(name)
        return working_extensions

    def _discover_plugins(self):
        """Merge installed rocker entry points with extensions available in-source."""
        plugins = list_plugins()
        for name in self.EXTENSIONS_TO_TEST:
            if name in plugins:
                continue
            plugin_class = self._load_local_extension(name)
            if plugin_class is not None:
                plugins[name] = plugin_class
        return plugins

    def _load_local_extension(self, extension_name):
        """Attempt to import a deps_rocker extension directly from the source tree."""
        module = None
        module_path = (
            Path(__file__).resolve().parent.parent
            / "deps_rocker"
            / "extensions"
            / extension_name
            / f"{extension_name}.py"
        )

        if module_path.exists():
            spec = importlib.util.spec_from_file_location(
                f"deps_rocker.extensions.{extension_name}.{extension_name}_local", module_path
            )
            if spec and spec.loader:
                module = importlib.util.module_from_spec(spec)
                sys.modules[spec.name] = module
                spec.loader.exec_module(module)

        if module is None:
            module_name = f"deps_rocker.extensions.{extension_name}.{extension_name}"
            try:
                module = importlib.import_module(module_name)
            except ModuleNotFoundError:
                return None

        for attr_name in dir(module):
            attr = getattr(module, attr_name)
            if (
                isinstance(attr, type)
                and issubclass(attr, SimpleRockerExtension)
                and attr is not SimpleRockerExtension
                and getattr(attr, "name", None) == extension_name
            ):
                return attr
        return None

    @contextmanager
    def _ros_jazzy_workspace(self):
        """Provide a temporary workspace dedicated to the ros_jazzy test run."""
        temp_dir = tempfile.TemporaryDirectory()  # pylint: disable=consider-using-with
        try:
            yield str(Path(temp_dir.name))
        finally:
            temp_dir.cleanup()

    def _build_base_cliargs(self, **additional_args):
        """Helper to build base cliargs with common settings"""
        cliargs = {
            "base_image": self.base_dockerfile_tag,
            "extension_blacklist": [],
            "strict_extension_selection": False,
        }
        cliargs.update(additional_args)
        return cliargs

    def run_extension_build_and_test(self, extension_name):
        """Shared logic to build, run, and check an extension, including running test.sh if present"""
        if extension_name not in self.working_extension_names:
            self.skipTest(f"Extension '{extension_name}' not available or not from deps_rocker")

        from rocker.core import RockerExtensionManager

        # Use rocker's extension manager to properly resolve and sort dependencies
        manager = RockerExtensionManager()
        manager.available_plugins.update(self.all_plugins)

        workspace_cm = (
            self._ros_jazzy_workspace() if extension_name == "ros_jazzy" else nullcontext(None)
        )

        with workspace_cm as workspace_root:
            cliargs = self._build_base_cliargs(**{extension_name: True})
            if extension_name == "odeps_dependencies":
                cliargs["deps"] = "deps_rocker.deps.yaml"
            if workspace_root:
                cliargs["auto"] = workspace_root

            # Let rocker's extension manager handle dependency resolution and sorting
            active_extensions = manager.get_active_extensions(cliargs)

            test_sh_path = Path(f"deps_rocker/extensions/{extension_name}/test.sh")
            extra_files = None
            extra_root_dir = None
            if extension_name == "ros_jazzy":
                test_sh_path = (
                    Path(__file__).resolve().parent / "fixtures" / "ros_jazzy_smoke_test.sh"
                )
                fixture_root = Path(__file__).resolve().parent / "test_package"
                extra_root_dir = "ros_jazzy_fixture"
                extra_files = {}
                for path in fixture_root.rglob("*"):
                    if path.is_file():
                        rel_path = path.relative_to(fixture_root)
                        extra_path = Path(extra_root_dir) / rel_path
                        extra_files[str(extra_path)] = path.read_text(encoding="utf-8")
            if test_sh_path.is_file():
                # Add the test script extension as the last extension
                extension_kwargs = {}
                if extra_files is not None:
                    extension_kwargs |= {
                        "extra_files": extra_files,
                        "extra_root_dir": extra_root_dir,
                    }
                active_extensions.append(
                    ScriptInjectionExtension(str(test_sh_path), **extension_kwargs)
                )
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

    def test_nvim_extension(self):
        self.run_extension_build_and_test("nvim")

    def test_pixi_extension(self):
        self.run_extension_build_and_test("pixi")

    # def test_urdf_viz_extension(self):
    #     self.run_extension_build_and_test("urdf_viz")

    def test_fzf_extension(self):
        self.run_extension_build_and_test("fzf")

    def test_gitui_extension(self):
        self.run_extension_build_and_test("gitui")

    def test_lazygit_extension(self):
        self.run_extension_build_and_test("lazygit")

    def test_cwd_extension(self):
        self.run_extension_build_and_test("cwd")

    def test_workdir_extension(self):
        self.run_extension_build_and_test("workdir")

    def test_ccache_extension(self):
        self.run_extension_build_and_test("ccache")

    def test_claude_extension(self):
        self.run_extension_build_and_test("claude")

    def test_codex_extension(self):
        self.run_extension_build_and_test("codex")

    def test_npm_extension(self):
        self.run_extension_build_and_test("npm")

    def test_cargo_extension(self):
        self.run_extension_build_and_test("cargo")

    def test_gemini_extension(self):
        self.run_extension_build_and_test("gemini")

    def test_spec_kit_extension(self):
        self.run_extension_build_and_test("spec_kit")

    def test_jquery_extension(self):
        self.run_extension_build_and_test("jquery")

    # def test_palanteer_extension(self):
    #     self.run_extension_build_and_test("palanteer")

    def test_conda_extension(self):
        self.run_extension_build_and_test("conda")

    # def test_isaac_sim_extension(self):
    #     self.run_extension_build_and_test("isaac_sim")

    def test_foxglove_extension(self):
        self.run_extension_build_and_test("foxglove")

    def test_ros_jazzy_extension(self):
        self.run_extension_build_and_test("ros_jazzy")

    def test_ros_underlay_extension(self):
        self.run_extension_build_and_test("ros_underlay")

    def test_vcstool_extension(self):
        self.run_extension_build_and_test("vcstool")

    def test_auto_extension(self):
        self.run_extension_build_and_test("auto")

    def _build_and_test_extensions(self, extensions_to_enable):
        """Helper to build and test a set of extensions together"""
        from rocker.core import RockerExtensionManager

        manager = RockerExtensionManager()

        # Build cliargs with specified extensions enabled
        cliargs = self._build_base_cliargs()
        for ext_name in extensions_to_enable:
            cliargs[ext_name] = True
            if ext_name == "odeps_dependencies":
                cliargs["deps"] = "deps_rocker.deps.yaml"

        # Let rocker's extension manager handle dependency resolution and sorting
        active_extensions = manager.get_active_extensions(cliargs)

        # Assert the order includes npm before codex/gemini for proper dependency ordering
        ext_names = [ext.get_name() for ext in active_extensions]
        if "npm" in ext_names and "codex" in ext_names:
            npm_idx = ext_names.index("npm")
            codex_idx = ext_names.index("codex")
            self.assertLess(npm_idx, codex_idx, "npm must come before codex")
        if "npm" in ext_names and "gemini" in ext_names:
            npm_idx = ext_names.index("npm")
            gemini_idx = ext_names.index("gemini")
            self.assertLess(npm_idx, gemini_idx, "npm must come before gemini")

        dig = DockerImageGenerator(active_extensions, cliargs, self.base_dockerfile_tag)
        build_result = dig.build()
        self.assertEqual(build_result, 0, "Extensions failed to build")
        with tempfile.NamedTemporaryFile(mode="r+") as tmpfile:
            run_result = dig.run(console_output_file=tmpfile.name)
            self.assertEqual(run_result, 0, "Extensions failed to run")
            tmpfile.seek(0)
            output = tmpfile.read()
            self.assertIn("Extension test complete", output)
        dig.clear_image()
        return active_extensions

    def test_npm_codex_gemini_ordering(self):
        """Test that npm is properly ordered before codex and gemini"""
        if "npm" not in self.working_extension_names:
            self.skipTest("npm extension not available")
        if "codex" not in self.working_extension_names:
            self.skipTest("codex extension not available")
        if "gemini" not in self.working_extension_names:
            self.skipTest("gemini extension not available")

        # Test that npm, codex, and gemini work together with proper ordering
        extensions_to_enable = ["npm", "codex", "gemini"]
        try:
            active_extensions = self._build_and_test_extensions(extensions_to_enable)
            # Additional verification that dependencies are properly resolved
            ext_names = [ext.get_name() for ext in active_extensions]
            self.assertIn("npm", ext_names)
            self.assertIn("codex", ext_names)
            self.assertIn("gemini", ext_names)
            self.assertIn("user", ext_names)  # Should be included as transitive dep
            self.assertIn("curl", ext_names)  # Should be included as transitive dep
        except Exception as e:
            self.fail(f"npm/codex/gemini ordering test raised an exception: {e}")

    def test_z_all_extensions_together(self):
        """Test that all extensions can be built and run together"""
        if not self.working_extension_names:
            self.skipTest("No working extensions found")

        # Get unique extension names from EXTENSIONS_TO_TEST that are available
        extensions_to_enable = [
            ext_name for ext_name in set(self.EXTENSIONS_TO_TEST) if ext_name in self.all_plugins
        ]

        try:
            self._build_and_test_extensions(extensions_to_enable)
        except Exception as e:
            self.fail(f"All extensions together raised an exception: {e}")


class ScriptInjectionExtension(SimpleRockerExtension):
    """Injects a test script (plus optional fixtures) into the Docker image and runs it."""

    name = "test_script"

    def __init__(
        self,
        script_path_or_content,
        is_content: bool = False,
        *,
        extra_files: dict[str, str] | None = None,
        extra_root_dir: str | None = None,
    ):
        self.context_name = "test.sh"
        self.extra_files = extra_files or {}
        if self.extra_files and not extra_root_dir:
            raise ValueError("extra_root_dir must be provided when extra_files are supplied")
        self.extra_root_dir = extra_root_dir

        if is_content:
            self.script_content = script_path_or_content
            self.script_path = None
        else:
            self.script_path = script_path_or_content
            self.script_content = None

    def get_snippet(self, cliargs):
        parts = []
        if self.extra_root_dir:
            parts.append(f"COPY {self.extra_root_dir}/ /tmp/{self.extra_root_dir}/")
        parts.append(f"COPY {self.context_name} /tmp/test.sh")
        parts.append("RUN chmod +x /tmp/test.sh")
        parts.append('CMD ["/tmp/test.sh"]')
        return "\n".join(parts)

    def get_files(self, cliargs):
        if self.script_content is not None:
            content = self.script_content
        else:
            with open(self.script_path, "r", encoding="utf-8") as f:
                content = f.read()
        if not content.lstrip().startswith("#!/"):
            raise RuntimeError(
                f"Error: test.sh script '{self.script_path}' is missing a shebang (e.g., #!/bin/bash) at the top."
            )
        files = {self.context_name: content}
        files.update(self.extra_files)
        return files


if __name__ == "__main__":
    unittest.main()
