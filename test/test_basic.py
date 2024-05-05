from unittest import TestCase
from deps_rocker.dependencies import Dependencies


class TestBasicClass(TestCase):
    def test_init(self):
        instance = Dependencies()

        # self.assertEqual(instance.int_var, 0)
