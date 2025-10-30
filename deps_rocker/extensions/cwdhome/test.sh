#!/bin/bash
set -e

echo "Testing cwdhome extension..."

# Test that the current directory is the home directory
if [ "$(pwd)" != "$HOME" ]; then
    echo "ERROR: Current directory is not home directory"
    echo "PWD: $(pwd)"
    echo "HOME: $HOME"
    exit 1
fi

# Test that we can see files from the host CWD
# The test framework should mount the host CWD to the container home
# We should be able to see at least some standard directories
echo "Current directory contents:"
ls -la

# Create a test file to verify write access
TEST_FILE="$HOME/cwdhome_test_file.txt"
echo "test content" > "$TEST_FILE"

if [ ! -f "$TEST_FILE" ]; then
    echo "ERROR: Could not create test file in home directory"
    exit 1
fi

# Clean up
rm "$TEST_FILE"

echo "cwdhome extension test completed successfully!"
