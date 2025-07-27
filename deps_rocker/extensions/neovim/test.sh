#!/bin/bash

set -e 


# Check that nvim (Neovim) is installed and prints its version
if ! command -v nvim &> /dev/null
then
    echo "Neovim (nvim) could not be found"
    exit 1
fi

nvim --version
if [ $? -eq 0 ]; then
    echo "Neovim is installed and working"
else
    echo "Neovim is installed but failed to run"
    exit 1
fi

# Optionally, run a minimal test (open and quit)
nvim --headless +q
echo "Neovim headless launch test passed"
