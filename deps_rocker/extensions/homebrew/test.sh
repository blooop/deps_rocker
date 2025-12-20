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

# Test brew config
echo "Testing brew config..."
brew config

echo "Homebrew extension test completed successfully!"
