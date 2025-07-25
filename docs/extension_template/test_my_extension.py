import pytest

# This is a template/example test. It is skipped in CI.
pytest.skip("Template test file -- skip in CI", allow_module_level=True)

# from my_package.extensions.my_extension import MyExtension


# class TestMyExtension(unittest.TestCase):
#     def test_my_extension(self):
#         ext = MyExtension()
#         self.assertEqual(ext.name, "my_extension")
#
#     def test_required_extensions(self):
#         ext = MyExtension()
#         required = ext.required({})
#         self.assertIn("curl", required)
