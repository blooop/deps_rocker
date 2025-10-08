#!/bin/bash
set -e

echo "Testing Neovim installation..."

if ! command -v nvim &> /dev/null; then
    echo "ERROR: nvim command not found"
    exit 1
fi

nvim --version

echo "Checking config mounts..."
if [ ! -d /root/.config/nvim ]; then
    echo "ERROR: /root/.config/nvim not mounted"
    exit 1
fi
if [ ! -d /root/.vim ]; then
    echo "ERROR: /root/.vim not mounted"
    exit 1
fi

echo "Neovim extension test completed successfully!"
