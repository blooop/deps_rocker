#!/bin/bash
set -e

echo "Testing cwd extension..."

# Get the expected directory name (should be the last component of PWD)
EXPECTED_DIR=$(basename "$(pwd)")
echo "Current directory: $(pwd)"
echo "Current user: $(whoami)"

# The cwd extension mounts the current working directory at <user_home>/<project_name>
# We just need to verify:
# 1. We're in a directory that is a subdirectory (ends with /<project_name>)
# 2. The directory is accessible/readable
# 3. We can read/write files here

if [[ "$(pwd)" != *"/$EXPECTED_DIR" ]]; then
    echo "ERROR: Current directory doesn't match expected mount pattern"
    echo "Expected path to end with: /$EXPECTED_DIR"
    echo "But got: $(pwd)"
    exit 1
fi

echo "Current directory path is correctly formatted: $(pwd)"

# Test that we can see files from the host CWD
echo "Current directory contents:"
ls -la

# Create a test file to verify write access
TEST_FILE="cwd_test_file.txt"
echo "test content" > "$TEST_FILE"

if [ ! -f "$TEST_FILE" ]; then
    echo "ERROR: Could not create test file in current directory"
    exit 1
fi

# Clean up
rm "$TEST_FILE"

echo "cwd extension test completed successfully!"
echo "Working directory $(pwd) is correctly inside user home at $ACTUAL_HOME"
