#!/bin/bash
set -e

echo "Testing poetry installation..."

if ! command -v poetry &> /dev/null; then
    echo "ERROR: poetry command not found"
    exit 1
fi

# Test basic functionality
poetry --version

# Test that poetry can create a new project (in a temp dir)
tmpdir=$(mktemp -d)
cd "$tmpdir"
poetry new test-project
cd test-project
poetry check

echo "poetry extension test completed successfully!"
