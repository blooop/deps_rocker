#!/bin/bash
set -e

echo "Testing Poetry installation..."

if ! command -v poetry &> /dev/null; then
    echo "ERROR: poetry command not found"
    exit 1
fi

poetry --version
echo "Poetry extension test completed successfully!"
