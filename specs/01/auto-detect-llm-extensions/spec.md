# Auto Extension Detection Spec

## Requirements
- Detect C/C++ source and header files (.c, .cc, .cpp, .cxx, .h, .hpp, .hxx) and add `ccache` to the detected extensions.
- Detect `package.xml` and add `ros_jazzy` to the detected extensions.
- Multiple file types should result in multiple extensions being detected (e.g., .cpp + package.json + pixi.toml → ccache, npm, pixi).
- Detection should be robust and work for files in the root directory.

## Acceptance Criteria
- All tests in `test/test_auto_extension.py` pass, especially those for ccache and ros_jazzy detection.
- No false positives for unrelated files.
- Detection logic is concise and maintainable.
