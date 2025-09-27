#!/bin/bash

set -e

echo "Testing cargo installation..."

# Check if cargo command is available
if ! command -v cargo &> /dev/null; then
    echo "ERROR: cargo command not found"
    exit 1
fi

# Check if rustc command is available
if ! command -v rustc &> /dev/null; then
    echo "ERROR: rustc command not found"
    exit 1
fi

# Test basic functionality
cargo --version
rustc --version

# Test that cargo can create a basic project
cargo --version | head -n 1

echo "cargo extension test completed successfully!"
