#!/bin/bash

set -e

# Add pixi bin to PATH (needed for non-interactive shells)
export PATH="$HOME/.pixi/bin:$PATH"

git --version
echo "git is installed and working"

# Test git-lfs
git lfs version
echo "git-lfs is installed and working"
