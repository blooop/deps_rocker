#!/bin/bash

set -e

# Check that fzf is installed from pixi and prints its version
if ! command -v fzf >/dev/null 2>&1; then
    echo "ERROR: fzf not found on PATH"
    exit 1
fi

fzf --version
echo "fzf is installed and working"

# Check that cdfzf function is defined in bashrc
if grep -q "cdfzf()" ~/.bashrc; then
    echo "cdfzf function found in bashrc"
else
    echo "ERROR: cdfzf function not found in bashrc"
    exit 1
fi
