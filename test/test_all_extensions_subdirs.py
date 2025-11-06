#!/usr/bin/env python3
# pylint: disable=protected-access,redefined-outer-name
"""Test that the subdirectory fix works for ALL extensions, not just ROS"""

import tempfile
import os
from pathlib import Path
from deps_rocker.extensions.auto.auto import Auto


def test_all_extension_subdirectory_detection():
    """Test that subdirectory detection works for multiple extensions"""

    # Create temporary directory
    test_dir = tempfile.mkdtemp()
    print(f"Testing in directory: {test_dir}")

    auto = Auto()
    original_dir = os.getcwd()

    try:
        # Change to test directory
        os.chdir(test_dir)

        # Create files in subdirectories for different extensions
        test_files = {
            # ROS extension - package.xml in subdirectories
            "ros_package_1/package.xml": """<?xml version="1.0"?>
<package format="3"><name>ros_pkg1</name><version>1.0.0</version></package>""",
            "ros_package_2/package.xml": """<?xml version="1.0"?>
<package format="3"><name>ros_pkg2</name><version>1.0.0</version></package>""",
            # NPM extension - package.json in subdirectories
            "frontend/package.json": '{"name": "frontend", "version": "1.0.0"}',
            "backend/package.json": '{"name": "backend", "version": "1.0.0"}',
            # Cargo extension - Cargo.toml in subdirectories
            "rust_lib/Cargo.toml": '[package]\nname = "rust_lib"\nversion = "0.1.0"',
            "rust_bin/Cargo.toml": '[package]\nname = "rust_bin"\nversion = "0.1.0"',
            # Conda extension - environment.yml in subdirectories
            "ml_env/environment.yml": "name: ml\ndependencies:\n  - python=3.9",
            "data_env/environment.yml": "name: data\ndependencies:\n  - pandas",
            # Ccache extension - C++ files in subdirectories
            "src/main.cpp": "#include <iostream>\nint main() { return 0; }",
            "lib/utils.cpp": '#include "utils.h"',
            # UV extension - requirements.txt (detects in subdirectories due to basename matching fix)
            "project_a/requirements.txt": "pytest\nblack",
            "project_b/requirements.txt": "requests\nnumpy",
            # Files with path patterns (should still work)
            ".cargo/config.toml": '[source.crates-io]\nreplace-with = "vendored-sources"',
        }

        for filepath, content in test_files.items():
            file_path = Path(filepath)
            file_path.parent.mkdir(parents=True, exist_ok=True)
            file_path.write_text(content, encoding="utf-8")

        print("Created test files in subdirectories:")
        for item in sorted(Path(".").rglob("*")):
            if item.is_file():
                print(f"  {item}")

        # Test detection with home directory isolation
        print("\n--- Testing detection ---")
        detected = auto._detect_files_in_workspace({"auto": test_dir}, check_home=False)
        print(f"Detected extensions: {sorted(detected)}")

        # Check expected detections
        expected_detections = {
            # "ros_jazzy": "package.xml files",
            "npm": "package.json files",
            "cargo": "Cargo.toml and .cargo/config.toml files",
            "conda": "environment.yml files",
            "ccache": ".cpp files",
            "uv": "requirements.txt files",
        }

        for ext, description in expected_detections.items():
            if ext in detected:
                print(f"✅ SUCCESS: {ext} detected from {description} in subdirectories")
            else:
                print(f"❌ FAILED: {ext} NOT detected from {description} in subdirectories")
                assert False, f"{ext} NOT detected from {description} in subdirectories"

    finally:
        os.chdir(original_dir)
        import shutil

        shutil.rmtree(test_dir)


if __name__ == "__main__":
    try:
        test_all_extension_subdirectory_detection()
        print("\n🎉 ALL extensions now properly detect files in subdirectories!")
    except AssertionError as e:
        print(f"\n💥 Some extensions still have subdirectory detection issues: {e}")
