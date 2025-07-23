import unittest
from my_package.extensions.my_extension import MyExtension


class TestMyExtension(unittest.TestCase):
    def test_my_extension(self):
        ext = MyExtension()
        self.assertEqual(ext.name, "my_extension")
        
    def test_required_extensions(self):
        ext = MyExtension()
        required = ext.required({})
        self.assertIn("curl", required)


if __name__ == "__main__":
    unittest.main()
