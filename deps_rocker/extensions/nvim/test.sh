#!/bin/bash

set -e 


# Check that nvim (nvim) is installed and prints its version
if ! command -v nvim &> /dev/null
then
    echo "nvim (nvim) could not be found"
    exit 1
fi

nvim --version
if [ $? -eq 0 ]; then
    echo "nvim is installed and working"
else
    echo "nvim is installed but failed to run"
    exit 1
fi

# Optionally, run a minimal test (open and quit)
nvim --headless +q
echo "nvim headless launch test passed"
