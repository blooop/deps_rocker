import unittest
from deps_rocker.extensions.detach.detach import Detach


class TestDetach(unittest.TestCase):
    def test_get_docker_args(self):
        # Test that the detach extension returns the correct Docker argument
        detach_ext = Detach()
        cliargs = {"detach": True}
        docker_args = detach_ext.get_docker_args(cliargs)

        self.assertEqual(docker_args, " --detach")

    def test_name(self):
        # Test that the extension has the correct name
        detach_ext = Detach()
        self.assertEqual(detach_ext.name, "detach")


if __name__ == "__main__":
    unittest.main()
