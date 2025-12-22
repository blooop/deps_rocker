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
brew --prefix

# Verify HOMEBREW_PREFIX is set
echo "HOMEBREW_PREFIX: $HOMEBREW_PREFIX"

echo "Homebrew extension test completed successfully!"
