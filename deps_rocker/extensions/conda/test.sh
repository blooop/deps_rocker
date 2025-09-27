#!/bin/bash

set -e

echo "Testing conda installation..."

if ! command -v conda &> /dev/null; then
    echo "ERROR: conda command not found"
    exit 1
fi

# Test conda is properly installed and working
conda --version

# Test conda can list environments
conda env list

# Test conda can install/list packages
conda list

echo "conda extension test completed successfully!"
