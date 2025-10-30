#!/bin/bash
set -e

echo "Testing cwd extension..."

# Get the expected directory name (should be the last component of PWD)
EXPECTED_DIR=$(basename "$(pwd)")
echo "Current directory: $(pwd)"
echo "Expected to be in: $HOME/$EXPECTED_DIR"

# Test that the current directory is inside the home directory
# PWD should be $HOME/<project_name>
if [[ "$(pwd)" != "$HOME"/* ]]; then
    echo "ERROR: Current directory is not inside home directory"
    echo "PWD: $(pwd)"
    echo "HOME: $HOME"
    exit 1
fi

# Test that we're in a subdirectory of home (not home itself)
if [ "$(pwd)" = "$HOME" ]; then
    echo "ERROR: Current directory should be $HOME/<project_name>, not $HOME itself"
    echo "PWD: $(pwd)"
    exit 1
fi

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
echo "Working directory $(pwd) is correctly inside home at $HOME"
