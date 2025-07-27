#!/bin/bash

set -e 


# Check that uv is installed and prints its version
if ! command -v fzf &> /dev/null
then
    echo "uv could not be found"
    exit
fi

fzf --version
echo "fzf is installed and working"
