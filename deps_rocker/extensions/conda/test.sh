#!/bin/bash

set -e

echo "Testing conda installation..."

# Initialize conda if not already on PATH
if ! command -v conda &> /dev/null; then
    if [ -f /opt/miniconda3/bin/conda ]; then
        export PATH="/opt/miniconda3/bin:$PATH"
        source /opt/miniconda3/etc/profile.d/conda.sh
    else
        echo "ERROR: conda not found at /opt/miniconda3/bin/conda"
        exit 1
    fi
fi

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
