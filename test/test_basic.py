import unittest
from unittest import TestCase
from deps_rocker.dependencies import Dependencies
import pytest

# pylint: disable=no-value-for-parameter


class TestBasicClass(TestCase):
    # def test_init(self):
    #     print(Path.cwd())
    #     instance = Dependencies("*.deps_test.yaml")
    #     instance.get_files(None)
    #     print(instance.get_snippet())

    @pytest.mark.skip
    def test_single(self):
        """a1.deps_test.yaml and a2.deps_test.yaml are the same. Check they result in the same output as each other, and also the same output when both files are loaded at the same time"""

        deps1 = Dependencies()
        deps1.setup_deps(dict(deps="a1.deps_test.yaml"))
        self.assertEqual([f.name for f in deps1.deps_files], ["a1.deps_test.yaml"])

        deps2 = Dependencies()
        deps2.setup_deps(dict(deps="a2.deps_test.yaml"))

        self.assertEqual([f.name for f in deps2.deps_files], ["a2.deps_test.yaml"])

        self.assertEqual(
            deps1.get_snippet(),
            deps2.get_snippet(),
            "both yaml files are the same so the snippets should be the same",
        )

        self.assertEqual(
            deps1.get_snippet(),
            deps2.get_snippet(),
            "Get snipped should be indemipotent",
        )

        deps_both = Dependencies()
        deps_both.setup_deps(dict(deps="a*.deps_test.yaml"))

        self.assertEqual(
            sorted([f.name for f in deps_both.deps_files]),
            ["a1.deps_test.yaml", "a2.deps_test.yaml"],
        )

        self.assertEqual(deps1.get_snippet(), deps_both.get_snippet())

    def test_filter(self):
        """a1.deps_test.yaml and a2.deps_test.yaml are the same. Check they result in the same output as each other, and also the same output when both files are loaded at the same time"""

        deps1 = Dependencies()
        deps1.setup_deps(dict(deps="a1.deps_test.yaml"))
        self.assertEqual([f.name for f in deps1.deps_files], ["a1.deps_test.yaml"])

        deps2 = Dependencies()
        deps2.setup_deps(dict(deps="a2.deps_test.yaml"))

        self.assertEqual([f.name for f in deps2.deps_files], ["a2.deps_test.yaml"])

        deps_both = Dependencies()
        deps_both.setup_deps(dict(deps="a*.deps_test.yaml"))

        self.assertEqual(
            sorted([f.name for f in deps_both.deps_files]),
            ["a1.deps_test.yaml", "a2.deps_test.yaml"],
        )

    @pytest.mark.skip
    def test_get_snippet(self):
        deps1 = Dependencies()
        deps1.setup_deps(dict(deps="a1.deps_test.yaml"))

        # deps1.
        for v in deps1.layers.values():
            v.to_snippet()

    def test_no_layer(self):
        deps = Dependencies()
        deps.setup_deps(dict(deps="no_layer.deps_test.yaml"))

        print(deps.layers)

        self.assertTrue("env" in deps.layers)
        self.assertTrue(deps.layers["env"].layer == "default")

    # def test_invalid_command(self):
    #     deps = Dependencies("invalid_command.deps_test.yaml")

    #     self.assertTrue("env" in deps.layers)
    #     self.assertTrue(deps.layers["env"].layer == "default")


if __name__ == "__main__":
    unittest.main()
