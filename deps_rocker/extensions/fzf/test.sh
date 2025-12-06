#!/bin/bash

set -e

# Add pixi bin to PATH (needed for non-interactive shells)
export PATH="$HOME/.pixi/bin:$PATH"

# Source bashrc for shell integration functions
source ~/.bashrc

fzf --version
echo "fzf is installed and working"

# Check that cdfzf function is defined in bashrc
if grep -q "cdfzf()" ~/.bashrc; then
    echo "cdfzf function found in bashrc"
else
    echo "ERROR: cdfzf function not found in bashrc"
    exit 1
fi
