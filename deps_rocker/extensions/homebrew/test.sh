#!/bin/bash

set -e

echo "Testing Homebrew installation..."

# Check if brew is installed and accessible
if ! command -v brew &> /dev/null; then
    echo "ERROR: brew command not found"
    exit 1
fi

# Check brew version
BREW_VERSION=$(brew --version)
echo "Homebrew version: $BREW_VERSION"

# Test basic brew functionality
echo "Testing brew list..."
brew list

# Test brew --prefix
echo "Testing brew --prefix..."
BREW_PREFIX=$(brew --prefix)
echo "Brew prefix: $BREW_PREFIX"

# Verify HOMEBREW_PREFIX is set
echo "HOMEBREW_PREFIX: $HOMEBREW_PREFIX"

# Test that the Homebrew directory is writable
echo "Testing Homebrew directory permissions..."
TEST_FILE="$BREW_PREFIX/test_write_permission.tmp"
if ! touch "$TEST_FILE" 2>/dev/null; then
    echo "ERROR: Cannot write to Homebrew directory $BREW_PREFIX"
    exit 1
fi
rm -f "$TEST_FILE"
echo "✓ Homebrew directory is writable"

# Test that we can create directories in Cellar
echo "Testing Cellar directory permissions..."
TEST_DIR="$BREW_PREFIX/Cellar/test_dir_permission.tmp"
if ! mkdir -p "$TEST_DIR" 2>/dev/null; then
    echo "ERROR: Cannot create directories in Cellar"
    exit 1
fi
rmdir "$TEST_DIR"
echo "✓ Cellar directory is writable"

echo "Homebrew extension test completed successfully!"
