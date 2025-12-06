#!/bin/bash
set -e

# Add pixi bin to PATH (needed for non-interactive shells)
export PATH="$HOME/.pixi/bin:$PATH"

echo "Testing deps-devtools installation..."

if ! command -v rg &> /dev/null; then
    echo "ERROR: ripgrep (rg) not found"
    exit 1
fi
if ! command -v fd &> /dev/null; then
    echo "ERROR: fd-find (fd) not found"
    exit 1
fi
if ! command -v fzf &> /dev/null; then
    echo "ERROR: fzf not found"
    exit 1
fi

# Test basic usage
rg --version
fd --version
fzf --version

echo "deps-devtools extension test completed successfully!"
