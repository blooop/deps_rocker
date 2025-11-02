# Plan

- Review `test/test_extensions_generic.py` and related ros_jazzy fixtures to see how the symlink is used.
- Adjust the test setup to avoid symlinking (e.g., use copy or configure the extension to operate in place) while keeping assertions equivalent.
- Run `pixi run ci`, address any failures, and keep iterating until it passes.
