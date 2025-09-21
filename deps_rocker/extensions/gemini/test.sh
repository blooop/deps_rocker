#!/bin/bash
set -e

echo "Testing gemini installation..."

if ! command -v gemini &> /dev/null; then
    echo "ERROR: gemini command not found"
    exit 1
fi

# Test basic functionality
gemini --version

echo "gemini extension test completed successfully!"
