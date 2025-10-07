#!/bin/bash
set -e

echo "Testing deps-devtools installation..."

if ! command -v rg &> /dev/null; then
    echo "ERROR: ripgrep (rg) not found"
    exit 1
fi
if ! command -v fdfind &> /dev/null; then
    echo "ERROR: fd-find (fdfind) not found"
    exit 1
fi
if ! command -v fzf &> /dev/null; then
    echo "ERROR: fzf not found"
    exit 1
fi

# Test basic usage
rg --version
fdfind --version
fzf --version || true

echo "deps-devtools extension test completed successfully!"
