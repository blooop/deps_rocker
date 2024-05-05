import unittest
from unittest import TestCase
from deps_rocker.dependencies import Dependencies
from pathlib import Path


class TestBasicClass(TestCase):
    def test_init(self):
        instance = Dependencies(Path("test"), "*.deps_test.yaml")
        # print(instance.get_commands_ordered())
        instance.get_files(None)

        # print("layers:")
        # [print(i) for i in ordered]


        print(instance.get_snippet(None))

        # self.assertEqual(instance.int_var, 0)


if __name__ == "__main__":

    unittest.main()
