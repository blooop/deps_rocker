#!/bin/bash

set -e

PATH="$HOME/.fzf/bin:$PATH"
# Check that uv is installed and prints its version
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
