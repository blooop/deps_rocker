#!/bin/bash

set -e

PATH="$HOME/.fzf/bin:$PATH"
# Check that uv is installed and prints its version
source ~/.bashrc
fzf --version
echo "fzf is installed and working"
