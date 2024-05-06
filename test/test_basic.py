import unittest
from unittest import TestCase
from deps_rocker.dependencies import Dependencies
from pathlib import Path


class TestBasicClass(TestCase):
    def test_init(self):
        print(Path.cwd())
        instance = Dependencies("*.deps_test.yaml")
        instance.get_files(None)
        print(instance.get_snippet())

    def test_single(self):
        """a1.deps_test.yaml and a2.deps_test.yaml are the same. Check they result in the same output as eachother, and also the same output when both files are loaded at the same time"""

        deps1 = Dependencies("a1.deps_test.yaml")
        self.assertEqual(deps1.deps_files, [Path("test/a1.deps_test.yaml")])

        deps2 = Dependencies("a2.deps_test.yaml")
        self.assertEqual(deps2.deps_files, [Path("test/a2.deps_test.yaml")])

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

        deps_both = Dependencies("a*.deps_test.yaml")

        self.assertEqual(
            deps_both.deps_files, [Path("test/a1.deps_test.yaml"), Path("test/a2.deps_test.yaml")]
        )

        print(deps1.get_snippet())
        print(deps_both.get_snippet())

        self.assertEqual(deps1.get_snippet(), deps_both.get_snippet())

    def test_no_layer(self):
        deps = Dependencies("no_layer.deps_test.yaml")

        print(deps.layers)

        self.assertTrue("env" in deps.layers)
        self.assertTrue(deps.layers["env"].layer == "default")

    def test_invalid_command(self):
        deps = Dependencies("invalid_command.deps_test.yaml")

        self.assertTrue("env" in deps.layers)
        self.assertTrue(deps.layers["env"].layer == "default")


if __name__ == "__main__":
    unittest.main()
