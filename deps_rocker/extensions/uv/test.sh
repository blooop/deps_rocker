#!/bin/bash

set -e 

# Check that uv is installed and prints its version
if ! command -v uv &> /dev/null
then
    echo "uv could not be found"
    exit
fi

uv --version
echo "uv is installed and working"
