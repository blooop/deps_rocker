#!/bin/bash
set -e

# Add pixi bin to PATH (needed for non-interactive shells)
export PATH="$HOME/.pixi/bin:$PATH"

echo "Testing lazygit installation..."

if ! command -v lazygit &> /dev/null; then
    echo "ERROR: lazygit command not found"
    exit 1
fi

# Test basic functionality
lazygit --version

echo "lazygit extension test completed successfully!"
